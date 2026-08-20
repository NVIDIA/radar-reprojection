# Radar Reprojection Design

## Scope

Radar Reprojection synthesizes radar scans for a simulated ego pose from logged radar scans and ego/actor trajectories. It is an offline log-replay component, not a radar propagation or material simulation.

## Architecture

The repository has three layers:

1. **`algorithm/`** — the dataset-independent C++ reprojection library. It preprocesses logged static and actor detections, selects source views, and reprojects detections into the requested radar pose. Its method is described in [`algorithm/README.md`](../algorithm/README.md).
2. **`proto/` and `server/`** — a thin gRPC translation layer. Protobuf messages mirror the algorithm structures; the server does not load datasets or know about clip identifiers.
3. **`examples/python/`** — dataset adapters and the user-facing client. `PhysicalAIAVAdapter` converts a PhysicalAI-AV clip into `LogData`; `iter_reproject_requests` produces requests for each ego-pose/radar pair.

The normal data flow is:

```text
dataset -> adapter -> LogData -> Initialize
                         |
ego/actor poses ---------+------> Reproject -> RadarScan
```

`Initialize` may replace previously loaded state. `Reproject` requires a successful initialization and a radar ID present in the initialized `LogData`. The server serializes both RPCs because the algorithm maintains mutable reprojection context.

## Wire contract

`proto/radar_reprojector.proto` is the authoritative interface. It exposes two unary RPCs:

- `Initialize` preprocesses `LogData` and algorithm parameters.
- `Reproject` synthesizes one scan for the requested timestamp, radar, ego pose, and actor poses.

Timestamps are signed 64-bit microseconds. Quaternions use Hamilton `(x, y, z, w)` order. Although `ActorState.from_world` is present on the wire, the server recomputes it from `to_world` to keep the transforms consistent.

`Initialize` is unary and can contain tens of megabytes of log data. The server defaults to a 256 MiB message limit; custom clients must configure matching gRPC send and receive limits. Transport is cleartext and does not provide TLS.

| Condition | gRPC status |
| --- | --- |
| `Reproject` before successful initialization | `FAILED_PRECONDITION` |
| Unknown radar ID or rejected initialization parameters/data | `INVALID_ARGUMENT` |
| Conversion or algorithm exception | `INTERNAL` |

## Dataset adaptation

The server and algorithm are dataset-independent. A dataset adapter implements
`RadarReprojectionDataAdapter`, producing:

- `LogData`: radar recordings plus ego and actor trajectories.
- `ReprojectorParam`: algorithm configuration.

The supplied PhysicalAI-AV mapping is documented in [SCHEMA.md](SCHEMA.md). For the extension workflow, see `SKILL.md` §8.

Actor states are interpolated to each ego timestamp. Actors whose trajectories do not bracket that timestamp are omitted rather than extrapolated. Requests are emitted in ascending ego-timestamp order and then sorted radar-ID order.

## Bazel workspace

The workspace uses Bazel modules (`MODULE.bazel`). The algorithm depends on Eigen and nlohmann-json; the wire layer adds protobuf and gRPC; Python targets use the `rules_python` toolchain and the root `requirements.txt`.

Eigen is provided from `$CONDA_PREFIX` through `third_party/eigen/eigen.bzl`. The conda environment also provides the `physical_ai_av` package used to download sample data. Python dependencies used by Bazel targets are resolved hermetically from `requirements.txt`.

The root targets expose the algorithm library and demo; `server/`, `proto/`, and `examples/python/` define the corresponding server, generated wire bindings, adapters, tests, and clients. Setup and commands are documented in the root `README.md`.

## Validation and performance

### Functional validation

The repository tests type conversion, server lifecycle and errors, trajectory interpolation, and PhysicalAI-AV adaptation. These tests establish functional correctness of the interfaces; they are not a quantitative measure of generated radar quality.

### Point-cloud quality

Point-cloud quality has no single definitive metric for this task. With no simulated trajectory deviation, the algorithm converges toward reconstruction of the original logged scan, so comparison with that same scan is primarily a consistency check. With trajectory deviation, the corresponding counterfactual radar scan was never recorded and therefore no direct ground truth exists. Even repeated observations at the same physical pose can differ substantially because radar returns are sparse and noisy.

As an approximate geometric measure, internal evaluation projects the point clouds into radial-distance maps and computes symmetric Chamfer distance. This measures bidirectional nearest-neighbor disagreement in metres, but does not fully capture detection credibility, RCS characteristics, velocity consistency, or downstream usefulness.

| Metric | original radar | reprojected radar |
| --- | ---: | ---: |
| Radar RDM distance (m) | 0.027 | 0.242 |

### End-to-end quality

End-to-end quality depends on the perception stack. In one NVIDIA internal perception-fusion evaluation, original camera videos were held fixed while original radar was replaced by reprojected radar:

| Metric | original radar (baseline) | reprojected radar (% of baseline) |
| --- | ---: | ---: |
| Radar object F1 — car | 0.933 | 0.826 (88.5%) |
| Radar object F1 — bike | 0.985 | 0.888 (90.2%) |
| Radar object F1 — pedestrian | 0.960 | 0.826 (86.0%) |

These values characterize that dataset, configuration, and perception stack; they are not general accuracy guarantees.

### Runtime

Runtime depends on hardware, the number and density of logged scans, visible occupied grid cells, configured source-view search size, and actor count. Internal tests run in real time on a moderate CPU machine, but no hardware-independent latency or throughput claim is made.
