# Radar Reprojection

A generic radar reprojection library for closed-loop sensor simulation: given logged radar scans plus ego/actor trajectories, synthesize realistic radar points from different ego poses in the closed-loop simulation.

**Status — Lightweight C++; CPU only; Battle tested.** The repo has been migrated from the NVIDIA NDAS production simulation system. It builds standalone with Bazel (bzlmod, Linux) and reads the open [PhysicalAI-Autonomous-Vehicles](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles) dataset format. It runs on CPU and accompanies with the GPU based main NuRec rendeering libraries during closed-loop simulation.

See [`algorithm/README.md`](algorithm/README.md) for the reprojection method, adaptive multi-frame sampling, and view-selection model.

## Project policies

- [License](LICENSE)
- [Contribution policy (external pull requests are not accepted)](CONTRIBUTING.md)
- [Code of Conduct](CODE_OF_CONDUCT.md)
- [Security Policy](SECURITY.md)

```
bazel build //...
bazel test  //...
```

## Environment setup

A conda env `radar_reprojection` is required: it provides Eigen C++ headers that the algorithm build links against (via `new_local_repository` against `$CONDA_PREFIX`), and the `physical_ai_av` Python package (the PyPI dataset SDK) used by `scripts/download_sample_clip.py`. See [`docs/DESIGN.md`](docs/DESIGN.md) §"Bazel workspace".

### One-time

```bash
conda create -n radar_reprojection python=3.11 -y
conda activate radar_reprojection
pip install physical_ai_av
conda install -c conda-forge eigen=3.4 -y
```

### Download sample data

The PhysicalAI-AV dataset is gated. Accept its license on Hugging Face, then authenticate before downloading:

```bash
hf auth login
```

```bash
conda activate radar_reprojection
python scripts/download_sample_clip.py                 # defaults
python scripts/download_sample_clip.py --radars all    # every radar in the clip
python scripts/download_sample_clip.py --clip auto     # pick a new clip
python scripts/download_sample_clip.py --top-k 5 --radars all               # 5 highest perception-quality clips
python scripts/download_sample_clip.py --top-k 5 --order lowest --radars all # 5 lowest-quality clips
python scripts/download_sample_clip.py --help
```

`--top-k K` downloads the K clips with the highest (or `--order lowest`)
perception `quality_score`, read from the sibling
[`PhysicalAI-Autonomous-Vehicles-NuRec`](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles-NuRec)
dataset's `clip_ratings_<version>.csv` (the base dataset carries no score).
Candidates are restricted to rated clips that exist here with the requested
radars + labels + calibration. Rated clips use the `srr_3`/`mrr_2` radar config,
so pair `--top-k` with `--radars all` — the default `radar_front_center_srr_0`
matches no rated clip.

### Building

The Bazel build needs `bazelisk` on PATH and `CONDA_PREFIX` pointing at the
`radar_reprojection` env. Source the helper once per shell, or set them yourself:

```bash
source scripts/env.sh
# equivalent to:
#   export PATH=$HOME/.local/bin:$PATH        # for bazelisk
#   export CONDA_PREFIX=$(conda info --base)/envs/radar_reprojection

bazel build //...
bazel test  //...
```

If `bazel` complains "CONDA_PREFIX env var is not set", you forgot to source
`scripts/env.sh` (or run the manual exports) in this shell.

### Running the gRPC server

`server/MainServer.cpp` produces a standalone gRPC server that exposes
`algorithm::RadarReprojector` over the wire. Build and run from the
repo root:

```bash
bazel build //server:radar_reprojection_server
bazel run   //server:radar_reprojection_server -- --address=0.0.0.0:50051
```

Server flags: `--address=HOST:PORT`, `--max-message-mb=N` (default 256), `--num-threads=N`, and `--help`.

Optimized build:

```bash
bazel build -c opt //server:radar_reprojection_server
```

Debug build:

```bash
bazel build --config=debug //server:radar_reprojection_server
```

Run all tests (algorithm + server + Python adapter):

```bash
bazel test //...
```

A trivial Python smoke client is at `examples/python/smoke_client.py`:

```bash
bazel run //examples/python:smoke_client -- --address=localhost:50051
```

A full clip can be driven end-to-end with `examples/python/run_clip.py`. It
loads a PhysicalAI-AV clip via `PhysicalAIAVAdapter`, calls `Initialize`,
then calls `Reproject` once per ego pose × radar:

```bash
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json
```

Add `--plot-output-dir=reproject_plots` to save each reprojected frame as
`reproject_plots/<radar_id>/ego_<index>.png` (throttle with `--plot-every=N`).
This works under `bazel run` because it uses the headless `Agg` backend.

### Live / on-the-fly plotting

`run_clip.py` can also open an interactive top-down window that updates on the
fly as each `(ego pose, radar)` is reprojected (`--show`), on top of — or
instead of — the saved PNGs. It uses **one shared window** that is cleared and
redrawn for every frame — one radar at a time, with radars selected by
`--radars` cycling through it. The view is fixed at ±100 m on both axes with a
grid, so it stays put instead of autoscaling frame to frame.

A live window needs an *interactive* matplotlib backend, which is **not
available under `bazel run`**: Bazel uses a hermetic, statically-linked Python
whose built-in `_tkinter` cannot be loaded by matplotlib's Tk backend, and no
other GUI toolkit is bundled. Run the client as plain Python from the
`radar_reprojection` conda env instead — its matplotlib drives the conda Tk
(`TkAgg`) and opens a real window. The C++ gRPC server still runs under Bazel.

One-time — install the Python client dependencies (grpcio, protobuf, numpy,
pyarrow, matplotlib) into the env:

```bash
conda activate radar_reprojection
pip install -r requirements.txt
```

Then run the server and the live client in two shells:

```bash
# Build once so the generated protobuf Python modules exist under bazel-bin/
source scripts/env.sh
bazel build //examples/python:run_clip

# Shell 1 — gRPC server (Bazel)
bazel run //server:radar_reprojection_server -- --address=0.0.0.0:50051

# Shell 2 — client with a live window (plain Python, from the repo root)
conda activate radar_reprojection
PYTHONPATH=.:bazel-bin python -m examples.python.run_clip \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json \
    --show
```

`PYTHONPATH=.:bazel-bin` puts the repo root (for the `examples.python` and
`proto` source packages) and Bazel's generated `proto/*_pb2.py` modules on the
import path — rebuild with `bazel build //examples/python:run_clip` after
editing `radar_reprojector.proto` so the generated modules stay in sync.
`--show` combines with `--plot-output-dir` / `--plot-every`. The final frame
stays on screen until you close the window.

If the client fails with confusing `UNAVAILABLE` / `UNIMPLEMENTED` errors,
check whether `http_proxy` / `https_proxy` are set in your shell — gRPC
honors those for outgoing connections. Unset them before running:

```bash
unset http_proxy https_proxy ALL_PROXY HTTP_PROXY HTTPS_PROXY
```

To plug in a different dataset format, subclass
`examples.python.adapter.RadarReprojectionDataAdapter` and pass your
implementation in place of `PhysicalAIAVAdapter`. Iteration over ego poses
and actor-state interpolation are handled by
`examples.python.reproject_iter.iter_reproject_requests`, which works with
any `pb.LogData`.
