# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Drive the radar-reprojection gRPC server end-to-end on a real clip.

Under Bazel (headless; save per-frame PNGs with --plot-output-dir):
    bazel run //examples/python:run_clip -- \\
        --address=localhost:50051 \\
        --dataset-root=sample_data \\
        --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \\
        --intrinsics-config=config/radar_intrinsics.json \\
        --plot-output-dir=reproject_plots --plot-every=20

For a live on-the-fly window (--show) run the client as plain Python from a
conda env with an interactive matplotlib backend -- `bazel run` uses a hermetic
interpreter whose Tk cannot drive a matplotlib window. See the README section
"Live / on-the-fly plotting":
    PYTHONPATH=.:bazel-bin python -m examples.python.run_clip \\
        --address=localhost:50051 \\
        --dataset-root=sample_data \\
        --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \\
        --intrinsics-config=config/radar_intrinsics.json \\
        --show
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

import grpc
import numpy as np

from examples.python import physical_ai_av_adapter as paa
from examples.python import reproject_iter as ri
from proto import radar_reprojector_pb2 as pb
from proto import radar_reprojector_pb2_grpc as pb_grpc


GRPC_OPTS = [
    ("grpc.max_receive_message_length", 256 * 1024 * 1024),
    ("grpc.max_send_message_length",    256 * 1024 * 1024),
]

# Seconds to pause per frame in --show mode, to let the GUI event loop refresh.
_LIVE_PAUSE_S = 0.001


def _bbox_corners_xy(pose: pb.Pose, half: pb.Vec3f) -> np.ndarray:
    """Return the 4-corner closed XY polygon of a bbox at `pose`, in the parent frame."""
    local = np.array([
        [-half.x, -half.y],
        [+half.x, -half.y],
        [+half.x, +half.y],
        [-half.x, +half.y],
        [-half.x, -half.y],
    ])
    R_xy = ri.quat_to_rot_matrix(pose.rotation)[:2, :2]
    t_xy = np.array([pose.translation.x, pose.translation.y])
    return local @ R_xy.T + t_xy


# Shared-window (ego-frame) half-extent in metres: a fixed +-100 m square that
# holds every radar's coverage at once, so the view stays put frame to frame.
_FIXED_LIM = 100.0


def _setup_axes(ax) -> None:
    """Apply the fixed top-down ego-frame view: labels, equal aspect, +-100 m, grid."""
    ax.set_xlabel("x (forward) [m]")
    ax.set_ylabel("y (left) [m]")
    ax.set_aspect("equal")
    ax.set_xlim(-_FIXED_LIM, _FIXED_LIM)
    ax.set_ylim(-_FIXED_LIM, _FIXED_LIM)
    ax.grid(True, alpha=0.3)


def _draw_ego_and_actors(ax, req: pb.ReprojectRequest) -> int:
    """Draw the ego bbox (at origin) and actor bboxes; return the actor count.

    Identical across all radars at one ego timestamp, so drawn once per ego.
    """
    eh = req.ego_state.bbox_half_size_m
    ego_x = [-eh.x, eh.x, eh.x, -eh.x, -eh.x]
    ego_y = [-eh.y, -eh.y, eh.y, eh.y, -eh.y]
    ax.plot(ego_x, ego_y, "b-", linewidth=2)
    ax.fill(ego_x, ego_y, alpha=0.2, color="blue")

    # Actor bboxes transformed into ego frame via ego.from_world * actor.to_world.
    for actor in req.actor_to_state.values():
        actor_in_ego = ri.compose_poses(req.ego_state.from_world, actor.to_world)
        corners = _bbox_corners_xy(actor_in_ego, actor.bbox_half_size_m)
        ax.plot(corners[:, 0], corners[:, 1], "r-", linewidth=1)
    return len(req.actor_to_state)


def _radar_detections_xy(req: pb.ReprojectRequest,
                         resp: pb.ReprojectResponse) -> tuple[list[float], list[float]]:
    """Project a reprojected scan's spherical detections into ego-frame XY."""
    R_re = ri.quat_to_rot_matrix(req.radar_param.to_ego.rotation)
    t_re = np.array([req.radar_param.to_ego.translation.x,
                     req.radar_param.to_ego.translation.y,
                     req.radar_param.to_ego.translation.z])
    xs: list[float] = []
    ys: list[float] = []
    for det in resp.scan.detections:
        d, az, el = det.distance_m, det.azimuth_rad, det.elevation_rad
        ce, se = math.cos(el), math.sin(el)
        ca, sa = math.cos(az), math.sin(az)
        ego_xyz = R_re @ np.array([d * ce * ca, d * ce * sa, d * se]) + t_re
        xs.append(ego_xyz[0])
        ys.append(ego_xyz[1])
    return xs, ys


class _FrameViewer:
    """Renders one (ego, radar) frame at a time into a shared top-down window.

    A single figure/axes is reused and cleared on every frame, so each
    reprojected scan is drawn on a fresh plot: the ego + actor bboxes plus that
    radar's detections, in a fixed +-100 m ego-frame view with a grid. The one
    window is shared across all radars selected by ``--radars`` (they cycle
    through it). When ``output_dir`` is set each frame is written to
    ``<output_dir>/<radar_id>/ego_<index>.png``. The caller only constructs a
    viewer when at least one of live/output is enabled.
    """

    def __init__(self, live: bool, output_dir: pathlib.Path | None) -> None:
        import matplotlib
        # Only force the headless Agg backend when purely saving PNGs; live
        # display needs whatever interactive backend is available.
        if not live:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        self._plt = plt
        self._live = live
        self._output_dir = output_dir
        self._radar_colors: dict[str, tuple] = {}   # radar_id -> stable color
        self._fig, self._ax = plt.subplots(figsize=(10, 10))
        mgr = self._fig.canvas.manager
        if mgr is not None:
            mgr.set_window_title("radar reprojection")
        if live:
            plt.ion()
            self._fig.show()

    def _color_for(self, radar_id: str) -> tuple:
        color = self._radar_colors.get(radar_id)
        if color is None:
            cmap = self._plt.get_cmap("tab10")
            color = cmap(len(self._radar_colors) % cmap.N)
            self._radar_colors[radar_id] = color
        return color

    def render(self, req: pb.ReprojectRequest, resp: pb.ReprojectResponse,
               ego_index: int) -> None:
        from matplotlib.lines import Line2D
        ax = self._ax
        ax.clear()                              # fresh plot every frame
        _setup_axes(ax)
        n_actors = _draw_ego_and_actors(ax, req)
        xs, ys = _radar_detections_xy(req, resp)
        color = self._color_for(req.radar_param.id)
        ax.scatter(xs, ys, s=4, color=color, zorder=3)

        handles = [Line2D([], [], color="blue", linewidth=2)]
        labels = ["ego"]
        if n_actors:
            handles.append(Line2D([], [], color="red", linewidth=1))
            labels.append(f"actors ({n_actors})")
        handles.append(Line2D([], [], marker="o", linestyle="",
                              color=color, markersize=4))
        labels.append(f"{req.radar_param.id} ({len(xs)})")
        ax.legend(handles=handles, labels=labels, loc="upper right",
                  fontsize="small")
        ax.set_title(f"{req.radar_param.id}  t={req.timestamp_us} us")

        if self._output_dir is not None:
            png = self._output_dir / req.radar_param.id / f"ego_{ego_index:05d}.png"
            png.parent.mkdir(parents=True, exist_ok=True)
            self._fig.savefig(png, dpi=80)
        if self._live:
            self._fig.canvas.draw_idle()
            self._plt.pause(_LIVE_PAUSE_S)

    def close(self, block: bool) -> None:
        # When live, keep the final frame on screen until the user closes it.
        if self._live and block:
            self._plt.ioff()
            self._plt.show()
        self._plt.close("all")


def _resolve_workspace_path(p: pathlib.Path) -> pathlib.Path:
    # Under `bazel run`, cwd is the runfiles dir; relative user-supplied paths
    # like `sample_data` won't resolve. Bazel exposes the original workspace
    # root via BUILD_WORKSPACE_DIRECTORY for exactly this case.
    if p.is_absolute():
        return p
    workspace = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    return pathlib.Path(workspace) / p if workspace else p


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", default="localhost:50051")
    p.add_argument("--dataset-root", type=pathlib.Path, required=True)
    p.add_argument("--clip", required=True, help="Clip UUID, e.g. 25cd4769-...")
    p.add_argument("--intrinsics-config", type=pathlib.Path, required=True,
                   help="Path to config/radar_intrinsics.json")
    p.add_argument("--radars", nargs="*", default=None,
                   help="Restrict to a subset of radar ids (default: all).")
    p.add_argument("--plot-output-dir", type=pathlib.Path, default=None,
                   help="If set, save each frame as "
                        "<dir>/<radar_id>/ego_<index>.png.")
    p.add_argument("--show", action="store_true",
                   help="Show an interactive top-down plot that updates on the "
                        "fly as each (ego_pose, radar) is reprojected. Can be "
                        "combined with --plot-output-dir.")
    p.add_argument("--plot-every", type=int, default=1,
                   help="Plot every N-th ego pose (default 1). Used when "
                        "--plot-output-dir and/or --show is set.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    dataset_root = _resolve_workspace_path(args.dataset_root)
    intrinsics_config = _resolve_workspace_path(args.intrinsics_config)
    plot_dir = (_resolve_workspace_path(args.plot_output_dir)
                if args.plot_output_dir is not None else None)
    viewer = (_FrameViewer(live=args.show, output_dir=plot_dir)
              if (args.show or plot_dir is not None) else None)

    adapter_obj = paa.PhysicalAIAVAdapter(
        dataset_root=dataset_root,
        clip_uuid=args.clip,
        intrinsics_config=intrinsics_config,
        radar_allowlist=args.radars or None,
    )

    print(f"Loading {args.clip} from {dataset_root} ...", flush=True)
    log_data = adapter_obj.load_log_data()
    radar_ids = sorted(log_data.log_radar_data)
    n_ego = len(log_data.log_actor_data.ego_trajectory.states)
    n_radars = len(radar_ids)
    print(f"Loaded: {n_ego} ego states, {n_radars} radars, "
          f"{len(log_data.log_actor_data.actor_to_trajectory)} actors", flush=True)
    print(f"Radars: {', '.join(radar_ids)}", flush=True)

    init_req = pb.InitializeRequest(
        param=adapter_obj.reprojector_param(),
        log_data=log_data,
    )

    total_dets = 0
    i_ego = 0
    last_ts = None

    with grpc.insecure_channel(args.address, options=GRPC_OPTS) as ch:
        stub = pb_grpc.RadarReprojectorServiceStub(ch)
        try:
            stub.Initialize(init_req)
        except grpc.RpcError as e:
            print(f"Initialize failed: {e.code().name} {e.details()}", file=sys.stderr)
            return 1

        for req in ri.iter_reproject_requests(log_data):
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
            n = len(resp.scan.detections)
            print(f"[ego {i_ego}/{n_ego}  t={req.timestamp_us}us] "
                  f"{req.radar_param.id} -> {n} detections")

            if viewer is not None and (i_ego - 1) % args.plot_every == 0:
                viewer.render(req, resp, i_ego)

            total_dets += n

    print(f"Done. {n_ego} ego poses x {n_radars} radars = {n_ego * n_radars} "
          f"reprojections, {total_dets} detections total.")

    if viewer is not None:
        # Block on the final live frame (if any) until the window is closed.
        viewer.close(block=args.show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
