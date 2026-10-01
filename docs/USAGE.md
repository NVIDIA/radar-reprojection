# Usage

This guide walks through environment setup, sample-data download, building,
testing, and running the radar reprojection server and example clients.

A quick note on the integration paths: the users can feel free to choose to integrate with the C++ library directly, or use the gRPC server/client approach for better insulation between the systems.

## Environment Setup

Create a Conda environment named `radar_reprojection`. The build uses this
environment for:

- Eigen C++ headers, linked by Bazel through `new_local_repository` against
  `$CONDA_PREFIX`
- The `physical_ai_av` Python package, used by
  `scripts/download_sample_clip.py` to access the PhysicalAI-AV dataset

For more detail, see `docs/DESIGN.md`, section "Bazel workspace".

## One-Time Setup

```bash
conda create -n radar_reprojection python=3.11 -y
conda activate radar_reprojection
pip install physical_ai_av
conda install -c conda-forge eigen=3.4 -y
```

## Download Sample Data

The PhysicalAI-AV dataset is gated. Accept the dataset license on Hugging Face,
then authenticate:

```bash
hf auth login
```

Download sample clips from the repo root:

```bash
conda activate radar_reprojection

# Default sample clip, all radars
python scripts/download_sample_clip.py

# Single radar subset
python scripts/download_sample_clip.py --radars radar_front_center_srr_0

# Pick a new clip automatically
python scripts/download_sample_clip.py --clip auto

# Download the 5 highest perception-quality clips
python scripts/download_sample_clip.py --top-k 5

# Download the 5 lowest perception-quality clips
python scripts/download_sample_clip.py --top-k 5 --order lowest

# Show all options
python scripts/download_sample_clip.py --help
```

`--top-k K` downloads the `K` clips with the highest perception
`quality_score`, or the lowest scores when combined with `--order lowest`.
Scores are read from the sibling
[`PhysicalAI-Autonomous-Vehicles-NuRec`](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles-NuRec)
dataset's `clip_ratings_<version>.csv`; the base dataset does not include
quality scores.

Candidate clips are restricted to clips that are rated and that include the
requested radars, labels, and calibration. Rated clips use the `srr_3`/`mrr_2`
radar configuration, which is covered by the default `--radars all`.
Explicitly requesting an `srr_0` radar, such as `radar_front_center_srr_0`,
does not match rated clips.

## Build and Test

The Bazel build requires `bazelisk` on `PATH` and `CONDA_PREFIX` pointing to
the `radar_reprojection` environment. Source the helper once per shell:

```bash
source scripts/env.sh
```

This is equivalent to:

```bash
export PATH=$HOME/.local/bin:$PATH
export CONDA_PREFIX=$(conda info --base)/envs/radar_reprojection
```

Build and test everything:

```bash
bazel build //...
bazel test  //...
```

If Bazel reports that `CONDA_PREFIX` is not set, source `scripts/env.sh` or run
the manual exports above in the current shell.

## Run the gRPC Server

`server/MainServer.cpp` builds a standalone gRPC server that exposes
`algorithm::RadarReprojector` over the wire.

From the repo root:

```bash
bazel build //server:radar_reprojection_server
bazel run   //server:radar_reprojection_server -- --address=0.0.0.0:50051
```

Supported server flags:

- `--address=HOST:PORT`
- `--max-message-mb=N` with default `256`
- `--num-threads=N`
- `--help`

Builds are optimized by default through `.bazelrc`. For a debug build:

```bash
bazel build --config=debug //server:radar_reprojection_server
```

Run all algorithm, server, and Python adapter tests:

```bash
bazel test //...
```

## Run Example Clients

### Smoke Client

A minimal Python smoke client is available at `examples/python/smoke_client.py`:

```bash
bazel run //examples/python:smoke_client -- --address=localhost:50051
```

### Full Clip Client

`examples/python/run_clip.py` drives a full clip end to end. It:

- Loads a PhysicalAI-AV clip through `PhysicalAIAVAdapter`
- Calls `Initialize`
- Calls `Reproject` once per logged radar scan
- Merges selected radar scan timestamps in ascending order
- Interpolates the ego pose to each scan timestamp

```bash
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json
```

`run_clip.py` is a sample driver, not a production pipeline. It demonstrates
the request flow, and the `--ego-shift` flag fabricates a constant pose offset
only to show how reprojection behaves.

For a closed-loop setup, write a pipeline that sends the real per-frame
simulated ego pose and per-actor poses in each `Reproject` request. To use a
different data source, subclass `RadarReprojectionDataAdapter` in
`examples/python/adapter.py` and reuse the pose/request helpers in
`examples/python/reproject_iter.py`.

## Save Reprojection Plots

Add `--plot-output-dir=reproject_plots` to save each reprojected frame:

```text
reproject_plots/<radar_id>/scan_<index>.png
```

Use `--plot-every=N` to throttle saved frames. Plotting works under
`bazel run` because the script uses matplotlib's headless `Agg` backend.

Add `--dummy` to skip the server and return the original logged scan for each
radar and timestamp. This identity baseline uses the same response format as
server output, so downstream plotting and summaries still run. Dummy-mode plots
overlay the logged ego trajectory.

## Reproject From a Shifted Ego Pose

Use `--ego-shift LAT_M LON_M HEADING_DEG` to request reprojection from an ego
pose offset relative to the logged ego pose.

The offset is expressed in the ego frame:

- `LAT_M`: lateral meters, positive left
- `LON_M`: longitudinal meters, positive forward
- `HEADING_DEG`: heading degrees, positive counter-clockwise

For example, replay the clip as if the ego vehicle were 2 m left with a
15-degree leftward heading:

```bash
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json \
    --ego-shift 2 0 15 \
    --plot-output-dir=reproject_plots
```

The shift only changes pose. The logged world-frame ego velocity is preserved:
the sensor rotates, but the motion through the world does not change. Actors
remain on their logged trajectories.

When plotting, each frame overlays the logged ego trajectory as a black dashed
line and the shifted ego trajectory in magenta. The clip ID and shift values
are included in the plot title.

A stacked visual comparison of these modes is shown at the top of the project
README.

## Live On-the-Fly Plotting

`run_clip.py` can open an interactive top-down plot window while scans are
being reprojected:

```bash
--show
```

The live plot can be used together with, or instead of, saved PNGs. It uses one
shared window that is cleared and redrawn for every frame, one radar at a time.
Radars selected by `--radars` cycle through the same window. The view is fixed
at +/-100 m on both axes with a grid, so it does not autoscale between frames.

Interactive plotting requires an interactive matplotlib backend. This is not
available under `bazel run`: Bazel uses a hermetic, statically linked Python,
whose built-in `_tkinter` cannot be loaded by matplotlib's Tk backend, and no
other GUI toolkit is bundled.

Run the C++ gRPC server under Bazel, but run the live Python client directly
from the `radar_reprojection` Conda environment. The Conda matplotlib install
can use Conda Tk through `TkAgg` and open a real window.

Install the Python client dependencies once:

```bash
conda activate radar_reprojection
pip install -r requirements.txt
```

Build once so Bazel generates the protobuf Python modules under `bazel-bin/`:

```bash
source scripts/env.sh
bazel build //examples/python:run_clip
```

Run the server in one shell:

```bash
bazel run //server:radar_reprojection_server -- --address=0.0.0.0:50051
```

Run the live client in a second shell from the repo root:

```bash
conda activate radar_reprojection
PYTHONPATH=.:bazel-bin python -m examples.python.run_clip \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json \
    --show
```

`PYTHONPATH=.:bazel-bin` adds both the repo root and Bazel-generated protobuf
modules to the import path. The repo root provides the `examples.python` and
`proto` source packages; `bazel-bin` provides generated `proto/*_pb2.py`
modules.

After editing `radar_reprojector.proto`, rebuild with:

```bash
bazel build //examples/python:run_clip
```

`--show` can be combined with `--plot-output-dir` and `--plot-every`. The final
frame stays on screen until you close the window.

## Troubleshooting

If the client fails with confusing `UNAVAILABLE` or `UNIMPLEMENTED` errors,
check whether proxy variables are set. gRPC honors these variables for outgoing
connections.

Unset them before running the client:

```bash
unset http_proxy https_proxy ALL_PROXY HTTP_PROXY HTTPS_PROXY
```

## Use Another Dataset Format

To plug in a different dataset format, subclass
`examples.python.adapter.RadarReprojectionDataAdapter` and pass your
implementation instead of `PhysicalAIAVAdapter`.

Request iteration and ego/actor state interpolation are handled by
`examples.python.reproject_iter.iter_reproject_requests`, which works with any
`pb.LogData`.
