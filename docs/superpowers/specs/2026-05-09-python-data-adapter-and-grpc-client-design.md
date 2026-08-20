# Python Data Adapter + gRPC Client — Design

Date: 2026-05-09
Status: Draft (pending implementation plan)
Authors: yahlu@nvidia.com

## 1. Overview & scope

Add a Python harness that drives the radar-reprojection gRPC server end-to-end
on a real clip:

1. An **abstract data adapter** (`RadarReprojectionDataAdapter`) that converts
   a user's dataset into the wire format (`pb.LogData` + `pb.ReprojectorParam`).
2. A **concrete adapter** (`PhysicalAIAVAdapter`) for the
   PhysicalAI-Autonomous-Vehicles dataset format the C++ `PhysicalAIAVLoader`
   already supports.
3. A **shared iterator** (`iter_reproject_requests`) that, given any
   `pb.LogData`, yields one `pb.ReprojectRequest` per `(ego_state, radar)`
   pair, interpolating actor states with slerp/lerp.
4. A **CLI client** (`run_clip.py`) that wires those three together: load a
   clip, call `Initialize`, then call `Reproject` for every ego pose × radar
   and print a per-frame summary.

The adapter is the only piece a downstream user has to subclass to plug in a
new dataset format. Iteration, interpolation, and gRPC orchestration are
shared and adapter-agnostic.

### Repository delta

| Path                                          | Status                                                              |
| --------------------------------------------- | ------------------------------------------------------------------- |
| `examples/python/adapter.py`                  | **New.** `RadarReprojectionDataAdapter` ABC + `default_reprojector_param()`. |
| `examples/python/physical_ai_av_adapter.py`   | **New.** `PhysicalAIAVAdapter` concrete implementation.             |
| `examples/python/reproject_iter.py`           | **New.** `iter_reproject_requests` + slerp/lerp utilities.          |
| `examples/python/run_clip.py`                 | **New.** End-to-end CLI client.                                     |
| `examples/python/test/`                       | **New.** Unit tests for `iter_reproject_requests` and `PhysicalAIAVAdapter`. |
| `examples/python/BUILD.bazel`                 | **Edited.** Adds `py_library` and `py_binary` targets.              |
| `examples/python/smoke_client.py`             | Unchanged. Stays as the lightweight in-process smoke test.          |
| `requirements.txt`                            | **Edited.** Adds `pyarrow` and `numpy`.                             |
| `README.md`                                   | **Edited.** Adds a "Run a real clip" subsection under the gRPC server quick-start. |

## 2. Adapter abstract base class

`examples/python/adapter.py`:

```python
from abc import ABC, abstractmethod
from proto import radar_reprojector_pb2 as pb

class RadarReprojectionDataAdapter(ABC):
    """Convert a dataset into the wire format consumed by RadarReprojectorService.

    Subclasses load their own data source and produce two proto messages: a
    LogData (radar logs + ego/actor trajectories) and a ReprojectorParam
    (algorithm tuning knobs). Iteration over ego poses for Reproject() is
    handled by reproject_iter.iter_reproject_requests, not by this class.
    """

    @abstractmethod
    def load_log_data(self) -> pb.LogData: ...

    @abstractmethod
    def reprojector_param(self) -> pb.ReprojectorParam: ...


def default_reprojector_param() -> pb.ReprojectorParam:
    """Sensible defaults for adapters that do not need to tune the algorithm.

    use_log_intrinsics=True, do_down_sampling=False, max_search_size=30,
    dump=False, bbox_scale_factor=(1,1,1), bbox_margin=(0,0,0).
    """
```

Two methods, not one combined `build_initialize_request()`, so subclasses can
override "what data" and "what algorithm settings" independently. The most
common pattern — custom data + default settings — is `super().reprojector_param()
= default_reprojector_param()`.

`load_log_data()` is expected to be idempotent and cheap on repeat calls
(implementations should cache).

## 3. PhysicalAIAVAdapter

`examples/python/physical_ai_av_adapter.py` mirrors `dataset/PhysicalAIAVLoader.cpp`
file-for-file: same parquet locations, same column names, same coordinate
conventions. The Python implementation is independent (no FFI) and uses
`pyarrow.parquet.read_table` + `numpy`.

```python
class PhysicalAIAVAdapter(RadarReprojectionDataAdapter):
    def __init__(
        self,
        dataset_root: pathlib.Path,
        clip_uuid: str,
        intrinsics_config: pathlib.Path,             # config/radar_intrinsics.json
        radar_allowlist: list[str] | None = None,    # None = all radars present
        reprojector_param: pb.ReprojectorParam | None = None,
    ): ...

    def load_log_data(self) -> pb.LogData: ...
    def reprojector_param(self) -> pb.ReprojectorParam: ...
```

### 3.1 Inputs read

| Input                                                           | Used for                                              |
| --------------------------------------------------------------- | ----------------------------------------------------- |
| `calibration/sensor_extrinsics/*.parquet`                       | Per-radar `to_ego` (filter by `clip_id == clip_uuid`). |
| `calibration/vehicle_dimensions/*.parquet`                      | Ego bbox half size from `(length, width, height)`.    |
| `labels/egomotion/<clip>.egomotion.parquet`                     | Ego trajectory: `(timestamp, qx,qy,qz,qw, x,y,z, vx,vy,vz)`. |
| `labels/obstacle.offline/<clip>.obstacle.offline.parquet`       | Per-actor trajectories keyed by `track_id`.           |
| `radar/<radar_id>/<clip>.<radar_id>.parquet`                    | One row per detection; group by `scan_index`.         |
| `intrinsics_config` (`config/radar_intrinsics.json`)            | Lookup by suffix: `radar_front_center_srr_0` → `srr_0`. |

Quaternion ordering: parquet stores `qx,qy,qz,qw`; this maps directly to
`pb.Quaternionf{x,y,z,w}` (Hamilton convention, `w` scalar last). No
component reordering at the API boundary.

### 3.2 Coordinate conventions (match C++ loader)

- **Ego trajectory** (`EgoTrajectoryBuilder.cpp`): `to_world` is built from
  parquet `q*` and `(x,y,z)`, `from_world = to_world.inverse()`. Velocity
  comes from parquet `vx,vy,vz` directly (no finite-diff).
- **Actor trajectory** (`ActorTrajectoryBuilder.cpp`): obstacle parquet
  rows are in **ego frame**. Convert to world via
  `actor_to_world = ego_to_world(t) * actor_to_ego`, where `ego_to_world(t)`
  is the ego pose interpolated to the obstacle's timestamp using the same
  slerp/lerp as `iter_reproject_requests` (§4). Velocity is finite-diffed
  per-actor after sorting by timestamp (mirrors `fillFiniteDiffVelocity` —
  central difference for interior, forward/backward at endpoints).
- **Radar detections** are recorded in the radar's own spherical frame, so
  no transform is applied — fields are copied 1:1 from parquet to
  `pb.RadarDetection`. `reference.radar_id`, `scan_index`, `detection_index`
  match the parquet `scan_index` and `detection_index`; `actor_id` is empty
  on load (the server's reprojection algorithm fills that in).

### 3.3 RadarParam derivation (per radar)

- `id` = the radar directory name (e.g. `radar_front_center_srr_0`).
- `intrinsics` = looked up by suffix in `radar_intrinsics.json`. Suffix is
  the trailing `srr_0` / `srr_3` / `mrr_2` / `imaging_lrr_1` segment.
- `to_ego` = extrinsics-parquet row whose `clip_id` matches `clip_uuid` and
  `sensor_name` matches the radar id. If absent, identity (with a warning).
- `max_num_returns` = max detection count over all that radar's scans
  (matches `PhysicalAIAVLoader.cpp:107-110`).

### 3.4 Caching and one-shot semantics

`load_log_data()` builds the proto once and caches the result on `self`.
Repeat calls return the same instance. Unlike the C++ loader, we do **not**
throw on a second `load()` — the Python adapter is allowed to be re-used by
multiple clients in the same process.

## 4. Iteration utility

`examples/python/reproject_iter.py`:

```python
def iter_reproject_requests(
    log_data: pb.LogData,
) -> Iterator[pb.ReprojectRequest]:
    """Yield one ReprojectRequest per (ego_state, radar) pair.

    Order: ego timestamps ascending, then radar ids in sorted order.
    Actor states are interpolated to each ego timestamp via slerp(rotation)
    + lerp(translation, velocity). Actors whose trajectory does not bracket
    the ego timestamp are skipped (no extrapolation).
    """
```

### 4.1 Interpolation

`slerp_actor_state(traj: pb.ActorTrajectory, t_us: int) -> pb.ActorState | None`:

- Locate bracketing states `s1, s2` such that `s1.timestamp_us <= t_us <= s2.timestamp_us`.
- If `t_us` falls outside `[traj.states[0].ts, traj.states[-1].ts]`, return `None`.
- Compute `ratio = (t_us - s1.ts) / max(s2.ts - s1.ts, 1)`.
- Translation, velocity, bbox_half_size, bbox_center: linear interp.
- Rotation: hand-rolled slerp on `(qx, qy, qz, qw)`, with sign flip if
  `dot(q1, q2) < 0` and a linear-fallback when the angle is small (< 1e-3).
  No scipy dependency.
- Recompute `from_world = to_world.inverse()` (the server does this anyway,
  but doing it here keeps the request self-consistent for offline tools).

`slerp_actor_state` is exported from `reproject_iter.py` and is also used by
`PhysicalAIAVAdapter` to interpolate the ego trajectory at obstacle
timestamps when converting actor poses from ego-frame to world-frame
(§3.2). One implementation, two callers.

### 4.2 Skip-vs-clamp policy

Out-of-range timestamps **skip** the actor. Rationale: extrapolating poses
across many seconds of missing data produces nonsense radar geometry; better
to omit the actor than to fabricate one. If a downstream user wants
clamp-to-endpoint behavior, they can wrap or override `iter_reproject_requests`.

### 4.3 Iteration order

For each ego state in ascending timestamp order, for each `radar_id` in
sorted order over `log_data.log_radar_data`, emit one `ReprojectRequest`.
For an N-pose, M-radar clip that's `N * M` total requests.

## 5. CLI client (`run_clip.py`)

Self-contained binary; usage:

```
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json \
    [--radars radar_front_center_srr_0 ...]
```

Flow:

```python
def main():
    args = parse_args()
    adapter = PhysicalAIAVAdapter(
        dataset_root=args.dataset_root,
        clip_uuid=args.clip,
        intrinsics_config=args.intrinsics_config,
        radar_allowlist=args.radars or None,
    )
    log_data = adapter.load_log_data()
    init_req = pb.InitializeRequest(
        param=adapter.reprojector_param(),
        log_data=log_data,
    )

    n_ego = len(log_data.log_actor_data.ego_trajectory.states)
    n_radars = len(log_data.log_radar_data)
    total_dets = 0
    i_ego = 0
    last_ts = None

    with grpc.insecure_channel(args.address, options=GRPC_OPTS) as ch:
        stub = pb_grpc.RadarReprojectorServiceStub(ch)
        stub.Initialize(init_req)

        for req in iter_reproject_requests(log_data):
            if req.ego_state.timestamp_us != last_ts:
                i_ego += 1
                last_ts = req.ego_state.timestamp_us
            try:
                resp = stub.Reproject(req)
            except grpc.RpcError as e:
                print(f"Reproject failed at ego {i_ego}/{n_ego} "
                      f"radar={req.radar_param.id}: {e.code().name} {e.details()}",
                      file=sys.stderr)
                return 1
            print(f"[ego {i_ego}/{n_ego}  t={req.timestamp_us}us] "
                  f"{req.radar_param.id} -> {len(resp.scan.detections)} detections")
            total_dets += len(resp.scan.detections)

    print(f"Done. {n_ego} ego poses x {n_radars} radars = {n_ego * n_radars} "
          f"reprojections, {total_dets} detections total.")
    return 0
```

`GRPC_OPTS` carries the same 256 MiB `grpc.max_{send,receive}_message_length`
overrides the smoke client uses — a real clip's `LogData` easily exceeds the
default 4 MiB cap.

## 6. Dependencies

`requirements.txt` gains:

```
pyarrow>=15.0
numpy>=1.26
```

`pyarrow` is required by `PhysicalAIAVAdapter` for parquet I/O. `numpy` is
used for vector math in `reproject_iter` and the adapter. Both are pure
runtime deps — the existing `grpcio==1.66.0` and `protobuf==5.29.0` stay.

If bazel-pip resolution of `pyarrow` (which has native extensions) is
problematic on the build host, the fallback is to import `pyarrow` lazily
inside `physical_ai_av_adapter.py` and document running `run_clip.py`
under the existing `physical_ai_av` conda env with `PYTHONPATH` pointing at
the bazel proto outputs. This fallback is not the preferred path; we will
attempt the bazel-pip route first.

`BUILD.bazel`:

```python
py_library(name = "adapter",
           srcs = ["adapter.py"],
           deps = ["//proto:radar_reprojector_py_proto"])

py_library(name = "reproject_iter",
           srcs = ["reproject_iter.py"],
           deps = ["//proto:radar_reprojector_py_proto",
                   requirement("numpy")])

py_library(name = "physical_ai_av_adapter",
           srcs = ["physical_ai_av_adapter.py"],
           deps = [":adapter",
                   ":reproject_iter",
                   "//proto:radar_reprojector_py_proto",
                   requirement("numpy"),
                   requirement("pyarrow")])

py_binary(name = "run_clip",
          srcs = ["run_clip.py"],
          deps = [":physical_ai_av_adapter",
                  ":reproject_iter",
                  "//proto:radar_reprojector_py_proto",
                  "//proto:radar_reprojector_py_grpc",
                  requirement("grpcio")])
```

## 7. Error handling

- **Adapter**: raises `FileNotFoundError` for missing parquets,
  `ValueError` for malformed rows, `KeyError` for an unknown radar suffix in
  `radar_intrinsics.json`. The client prints the exception and exits 1.
- **gRPC**: client catches `grpc.RpcError`, prints `code().name` +
  `details()`, exits 1. No retries.
- **Out-of-range actor timestamps**: silently skipped by
  `iter_reproject_requests` (§4.2). Counts are not surfaced in the per-frame
  summary; if a future debugging need arises we can add a `--verbose`
  toggle.

## 8. Testing

| Test                                       | Type        | What it covers                                                  |
| ------------------------------------------ | ----------- | --------------------------------------------------------------- |
| `reproject_iter_test.py`                   | unit        | Synthetic 3-state ego trajectory + 1 actor with mismatched timestamps. Verify request count = `n_ego × n_radars`, request order, interpolated actor pose lies between bracketing source poses, out-of-range actors are skipped. |
| `physical_ai_av_adapter_test.py`           | integration | Load `sample_data/` for the default clip, assert `>0` ego states, `>0` actors, all 8 radars present, `RadarParam.intrinsics` matches `radar_intrinsics.json` lookup. |
| `run_clip_smoke.sh` (manual)               | end-to-end  | Spin up server in background, run `run_clip.py` against `sample_data`, expect non-zero detection count and exit 0. Documented in README; not run in CI. |

Unit tests use pytest discovery; bazel `py_test` targets wrap them. The
integration test requires the `physical_ai_av` conda env (for parquet
I/O); we tag it `manual` so it doesn't block `bazel test //...`.

## 9. Out of scope

- Streaming Reproject RPC. The current proto is unary; one RPC per
  `(ego_state, radar)`. Streaming is a future optimization, not part of
  this work.
- Saving reprojected scans to disk. The client prints summary lines only.
  When a real downstream consumer (evaluator, visualizer) appears, we add
  an `--output` flag at that point — not before.
- Latency benchmarking. `run_clip.py` measures nothing; if we want
  benchmarks later, add a separate `bench_clip.py` rather than overloading
  the demo client.
- Custom adapter examples. The PhysicalAI adapter is the only concrete
  implementation in this work; documenting "how to write your own adapter"
  is a README addition, not a separate skeleton in `examples/`.
