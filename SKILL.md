---
name: radar-reprojection
description: Build, test, and run the standalone radar-reprojection tool in this repo (DriveWorks-free, Bazel/bzlmod, amd64 Linux). Use when setting up the conda env, downloading PhysicalAI-AV sample data, building/testing with Bazel, running the gRPC server, driving a clip with the Python client (including live/on-the-fly plotting), or understanding the algorithm/proto/python layers. Keywords: radar reprojection, bazel build, conda radar_reprojection, run_clip, --show, download_sample_clip, --top-k, perception quality score, NuRec clip_ratings, gRPC server, sample_data, RadarReprojector, Initialize, Reproject.
---

# Radar Reprojection — self-contained setup, build & run guide

This is everything needed to work in this repository without reading any other
file. Work from the repo root.

## What this project is

A standalone, DriveWorks-free, Bazel-native (bzlmod, amd64 Linux) **radar
log-replay reprojection tool**. Given logged radar scans plus ego/actor
trajectories, it synthesizes what the same radars would have seen from a
*different* (simulated) ego pose. It reads the open
[PhysicalAI-Autonomous-Vehicles](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles)
dataset format. It was extracted from an NVIDIA NDAS monorepo and now builds
standalone.

### Architecture — three layers (boundary strictly enforced)

1. **`algorithm/` — pure C++ reprojection.** Eigen + nlohmann-json only (no
   DriveWorks, no protobuf, no parquet). Core type shapes:
   - `LogData` = `LogRadarData` (per-radar `LogRecording{ RadarParam, scans[] }`)
     + `LogActorData` (ego trajectory + per-actor trajectory).
   - `ReprojectorParam` — tuning knobs: `useLogIntrinsics`, `doDownSampling`,
     `bboxScaleFactor`/`bboxMargin` (actor bbox dilation for occlusion/noise),
     `maxSearchSize` (default 30; set 1 to mimic v1 behavior), `dump` +
     `dumpFolder` (best-effort: an unusable folder warns on stderr and skips
     dumps; `initialize()` still succeeds).
   - `ReprojectInput` = `{ timestampUs, radarParam, egoState, actorToState }`.
     The caller interpolates `ActorState`s (slerp) before calling `reproject()`.
   - `RadarScan{ timestampUs, detections[] }`; each `RadarDetection` carries a
     `ReferenceIndex{ radarId, scanIndex, detectionIndex, actorId }` tracing it
     back to the originating log sample — the **only** link to provenance;
     preserve it through post-processing.
   - Pipeline inside `RadarReprojector`: `initialize(param, logData)`
     pre-processes every log frame into a world-frame `StaticPointcloud`
     (`RadarView` fan + `GridLod` spatial index) or per-actor `ActorPointcloud`
     (points labelled static vs. actor via dilated actor bboxes); `reproject(input)`
     builds a new `RadarView` at the requested pose, walks the static `GridLod`
     for the best-matching log view per cell, reprojects matched points into the
     new sensor frame, does the same per actor, then optionally downsamples.
     `RadarView` is a 2D fan-shape abstraction (FoV containment, IoU, view
     similarity — the scoring used in view selection); `GridLod` is a
     level-of-detail grid avoiding O(N²) point lookups.
   - Detailed method: [`algorithm/README.md`](algorithm/README.md).

2. **`proto/` + `server/` — gRPC wire layer.** `proto/radar_reprojector.proto`
   mirrors the algorithm structs one-to-one. Two RPCs: `Initialize` and
   `Reproject`. The server is a thin translation layer (protobuf ↔ algorithm
   structs) — dataset-agnostic, no clip loading. `log_radar_data` is a
   `map<string, LogRecording>` keyed by radar id.

3. **`examples/python/` — dataset adapters + CLI (the user-facing path).**
   - `adapter.py` — `RadarReprojectionDataAdapter` ABC + `default_reprojector_param()`.
   - `physical_ai_av_adapter.py` — `PhysicalAIAVAdapter` loads a PhysicalAI-AV
     clip from parquet via pyarrow and emits a `pb.LogData`.
   - `reproject_iter.py` — pose-math helpers (`slerp_quat`, `lerp_pose`,
     `compose_poses`, `invert_pose`, `quat_to_rot_matrix`, …), `slerp_actor_state`,
     and `iter_reproject_requests(log_data)` which yields one `pb.ReprojectRequest`
     per logged radar scan, following the radars' scan timestamps merged in
     ascending order (ties broken by radar id); the ego state is interpolated
     to each scan timestamp.
   - `run_clip.py` — CLI client: load clip → `Initialize` → `Reproject` per
     logged radar scan, with optional PNG / live plotting.
   - `smoke_client.py` — minimal connectivity client.

### Repo layout

```
algorithm/      pure C++ reprojection lib (+ JsonIO)
proto/          radar_reprojector.proto (+ generated *_pb2 under bazel-bin/)
server/         gRPC server (thin protobuf<->algorithm translation)
examples/python/ adapter framework + run_clip / smoke_client + tests
config/         radar_intrinsics.json (per-radar-type range/FoV/resolution)
scripts/        env.sh (per-shell setup) + download_sample_clip.py
sample_data/    downloaded clip fixture (may be empty on fresh checkout)
third_party/    eigen.bzl (Eigen via $CONDA_PREFIX)
docs/           design and data-schema references
MODULE.bazel    bzlmod deps; pip.parse(requirements_lock="//:requirements.txt")
requirements.txt hermetic pip deps for Bazel; also the client pip list for conda
```

## 0. Prerequisites

- `conda` (miniconda/anaconda) on the machine.
- `bazelisk` installed at `~/.local/bin` (put on PATH by `scripts/env.sh`).
- Linux amd64. A GUI / `$DISPLAY` is only needed for live plotting (`--show`).

## 1. One-time environment setup

A conda env named **`radar_reprojection`** (Python 3.11) is required. It provides
the Eigen C++ headers the algorithm build links against
(`$CONDA_PREFIX/include/eigen3`, via `third_party/eigen/eigen.bzl`) and the
`physical_ai_av` PyPI package used to download sample clips. All other Python
deps (numpy, pyarrow, matplotlib, grpcio, protobuf) come from Bazel's hermetic
pip — **not** the conda env — during `bazel` builds.

```bash
conda create -n radar_reprojection python=3.11 -y
conda activate radar_reprojection
pip install physical_ai_av                    # dataset SDK (for downloading clips)
conda install -c conda-forge eigen=3.4 -y     # Eigen headers for the C++ build
```

Optional — only needed to run the Python client **outside Bazel** (e.g. for live
plotting, §6). Installs the client runtime into the same env:

```bash
pip install -r requirements.txt               # grpcio, protobuf, numpy, pyarrow, matplotlib
```

## 2. Per-shell environment

Every Bazel invocation needs `bazelisk` on PATH and `CONDA_PREFIX` pointing at
the env. Source the helper once per shell:

```bash
source scripts/env.sh
```

It exports `PATH="$HOME/.local/bin:$PATH"` and
`CONDA_PREFIX=$(conda info --base)/envs/radar_reprojection`. If Bazel errors with
`CONDA_PREFIX env var is not set`, you didn't source it in this shell. If it
warns that `eigen3` headers are missing, re-run the `conda install eigen=3.4` step.

## 3. Prepare data

Sample clips download into `sample_data/` (which may be empty on a fresh
checkout). The integration test `//examples/python:physical_ai_av_adapter_test`
requires it, so populate it before `bazel test //...`.

The dataset is gated. Accept its license on Hugging Face and run `hf auth login` before downloading.

```bash
conda activate radar_reprojection
python scripts/download_sample_clip.py                 # defaults (below)
python scripts/download_sample_clip.py --radars radar_front_center_srr_0     # a subset of radars
python scripts/download_sample_clip.py --clip auto     # pick a new clip
python scripts/download_sample_clip.py --top-k 5                 # 5 highest-quality clips
python scripts/download_sample_clip.py --top-k 5 --order lowest  # 5 lowest-quality clips
python scripts/download_sample_clip.py --help
```

Defaults: `--clip 25cd4769-5dcf-4b53-a351-bf2c5deb6124`, `--radars all`
(every radar present in the clip; for the sample clip that is 9 SRR radars:
front-center, side-left/right, corner-front/rear-left/right, rear-left/right),
`--out sample_data`.

`--top-k K` selects the K clips with the highest (or `--order lowest`) perception
`quality_score` instead of a single `--clip`. Scores come from the sibling
`nvidia/PhysicalAI-Autonomous-Vehicles-NuRec` dataset's
`clip_ratings_<version>.csv` (the base dataset has no score); candidates are
restricted to rated clips available here with the requested radars + labels +
calibration. Rated clips use the `srr_3`/`mrr_2` radar config, covered by the
default `--radars all`; explicitly requesting an `srr_0` radar
(e.g. `radar_front_center_srr_0`) matches no rated clip.

## 4. Build & test

```bash
source scripts/env.sh
bazel build //...
bazel test  //...          # algorithm + server + Python adapter tests
```

Builds are optimized (`-c opt`) by default via `.bazelrc`; it applies to every
target — libs, binaries, and tests. Overrides:

```bash
bazel build --config=debug  //server:radar_reprojection_server   # -c dbg -O0 -g
bazel build -c fastbuild    //server:radar_reprojection_server   # unoptimized fastbuild
```

Key targets:
- `//:algorithm` — algorithm-only C++ static lib (Eigen + nlohmann-json + logger).
- `//server:radar_reprojection_server` — gRPC server.
- `//examples/python:smoke_client` — minimal in-process smoke client.
- `//examples/python:run_clip` — full CLI driver.
- `//examples/python:reproject_iter_test`, `//examples/python:physical_ai_av_adapter_test`
  — Python unit + integration tests.

## 5. Run (under Bazel)

Start the gRPC server in one shell:

```bash
source scripts/env.sh
bazel run //server:radar_reprojection_server -- --address=0.0.0.0:50051
```

Drive a clip end-to-end in another shell (loads via `PhysicalAIAVAdapter`, calls
`Initialize`, then `Reproject` per logged radar scan — the selected radars'
scan timestamps are merged in ascending order and the ego pose is interpolated
to each one):

```bash
source scripts/env.sh
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json
```

`run_clip` flags:
- `--radars <id> [<id> …]` — restrict to a subset of radar ids (default: all loaded).
- `--ego-shift LAT_M LON_M HEADING_DEG` — reproject from a pose offset from the
  logged ego pose, given in ego frame: lateral metres (+left), longitudinal
  metres (+forward), heading degrees (+CCW). A pure pose shift: the logged
  world-frame ego velocity is kept; actors stay on their logged trajectories.
  When plotting, the logged (black dashed) and shifted (magenta) ego
  trajectories are overlaid on every frame.
- `--dummy` — no server needed: each request is answered with the exact
  original logged scan for that radar + timestamp, in the regular response
  format (identity baseline; plotting and summaries work unchanged). Plots
  overlay the logged ego trajectory.
- `--plot-output-dir <dir>` — save a per-frame top-down PNG to
  `<dir>/<radar_id>/scan_<index>.png` (headless `Agg`; works under `bazel run`).
- `--plot-every N` — only plot every N-th scan.
- `--show` — live interactive window (§6; **does not work under `bazel run`**).

Shifted-pose replay example — reproject as if the ego drove 2 m further left
with a 15-degree leftward heading, saving PNGs with both trajectories overlaid:

```bash
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json \
    --ego-shift 2 0 15 --plot-output-dir=reproject_plots
```

On load it prints the ego/radar/scan/actor counts and the radar id list. Connectivity
check: `bazel run //examples/python:smoke_client -- --address=localhost:50051`.

## 6. Live / on-the-fly plotting (`--show`) — run the client outside Bazel

`--show` opens one shared top-down window (ego at origin, actor boxes, detection
points), fixed at **±100 m** on both axes with a grid, **cleared and redrawn per
frame** (one radar at a time; radars from `--radars` cycle through it).

It needs an **interactive matplotlib backend, which is NOT available under
`bazel run`**: Bazel's hermetic python-build-standalone interpreter has a
statically-linked `_tkinter` that matplotlib's Tk backend cannot `dlopen`, and no
Qt/GTK/wx toolkit is bundled → matplotlib falls back to non-interactive `Agg` and
no window appears. The fix is to run the **client** as plain Python from the conda
env (its matplotlib has a working shared `_tkinter` → TkAgg). The C++ server still
runs under Bazel.

```bash
# One-time: client deps in the env (§1 optional step)
#   conda activate radar_reprojection && pip install -r requirements.txt

# Build once so the generated protobuf Python modules exist under bazel-bin/
source scripts/env.sh
bazel build //examples/python:run_clip

# Shell 1 — server (Bazel)
bazel run //server:radar_reprojection_server -- --address=0.0.0.0:50051

# Shell 2 — client with a live window (plain Python, from repo root)
conda activate radar_reprojection
PYTHONPATH=.:bazel-bin python -m examples.python.run_clip \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json \
    --show
```

- `PYTHONPATH=.:bazel-bin`: `.` supplies the source packages (`examples.python`,
  `proto`); `bazel-bin` supplies Bazel's generated `proto/*_pb2.py` /
  `*_pb2_grpc.py`. Neither `proto/` dir has `__init__.py`, so Python merges them
  as a namespace package.
- Run as a module (`python -m examples.python.run_clip`), **not**
  `python examples/python/run_clip.py`, or the absolute imports break.
- After editing `proto/radar_reprojector.proto`, rerun
  `bazel build //examples/python:run_clip` so the generated modules stay in sync.
- `--show` combines with `--plot-output-dir` / `--plot-every`; the final frame
  stays on screen until you close the window.

## 7. Gotchas / troubleshooting

- **`CONDA_PREFIX env var is not set`** → `source scripts/env.sh` in this shell.
- **Missing `eigen3` headers** → `conda install -c conda-forge eigen=3.4 -y` in the env.
- **`sample_data/` empty / adapter test fails** → run `scripts/download_sample_clip.py`
  (downloads every radar of the sample clip by default).
- **`no calibration rows for clip <uuid> under .../calibration/...`** → that clip's
  calibration chunk is not in this dataset root; re-run `download_sample_clip.py --clip <uuid>`.
  Several clips may share one root: every `*.chunk_NNNN.parquet` under `calibration/` is searched.
- **`[W] dump folder '<path>' is missing or not a directory`** → `dump=true` but the
  folder does not exist or is a regular file. Dumps are skipped and reprojection
  continues; create the directory before `Initialize` to get `log_data.json` etc.
- **gRPC `UNAVAILABLE` / `UNIMPLEMENTED`** → unset proxies before running:
  `unset http_proxy https_proxy ALL_PROXY HTTP_PROXY HTTPS_PROXY`.
- **`--show` shows no window / "FigureCanvasAgg is non-interactive"** → you ran it
  under `bazel run`; use the conda plain-Python path in §6.
- **`ModuleNotFoundError: proto`** when running plain Python → build
  `//examples/python:run_clip` first and include `bazel-bin` in `PYTHONPATH`.
- **Quaternion ordering** on the wire is `pb.Quaternionf{x,y,z,w}` = `Eigen::Quaternionf`
  (Hamilton convention, `w` scalar); parquet uses the same order, so the adapter
  copies fields without reordering.
- **Out-of-window states are silently dropped**: `iter_reproject_requests` skips
  scans whose timestamp falls outside the ego trajectory's range, and (like the
  adapter) skips actors whose trajectory does not bracket the scan timestamp —
  no extrapolation.
- **`ReferenceIndex`** on each detection is the only provenance link back to the
  source log sample; preserve it through post-processing.
- The algorithm lib pins `-std=c++20` and disables the `maybe-uninitialized`
  warning for Eigen under gcc13. The JSON dump schema
  (`log_data.json` / `initialize.json` / `reproject_<radar>_<ts>.json`, via
  `algorithm/JsonIO.{hpp,cpp}`) is **not versioned** — keep readers in sync.

## 8. Extending to another dataset

Subclass `examples.python.adapter.RadarReprojectionDataAdapter` and emit a
`pb.LogData`; use it in place of `PhysicalAIAVAdapter`. Scan iteration and
ego/actor-state interpolation are handled by
`examples.python.reproject_iter.iter_reproject_requests`, which works with any
`pb.LogData`. The algorithm and server stay dataset-agnostic — clients send
`pb.LogData` over the wire.
