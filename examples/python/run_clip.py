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
import contextlib
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


def _world_to_ego_xy(from_world: pb.Pose, pts_world: np.ndarray) -> np.ndarray:
    """Transform Nx3 world-frame points into ego frame, returning Nx2 XY."""
    R = ri.quat_to_rot_matrix(from_world.rotation)
    t = np.array([from_world.translation.x,
                  from_world.translation.y,
                  from_world.translation.z])
    return (pts_world @ R.T + t)[:, :2]


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
    """Draw the ego bbox (at origin) and actor bboxes; return the actor count."""
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
    ``<output_dir>/<radar_id>/scan_<index>.png``, where ``index`` is the
    request's position in the merged scan schedule. The caller only constructs
    a viewer when at least one of live/output is enabled.
    """

    def __init__(self, live: bool, output_dir: pathlib.Path | None,
                 header: str = "") -> None:
        import matplotlib
        # Only force the headless Agg backend when purely saving PNGs; live
        # display needs whatever interactive backend is available.
        if not live:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        self._plt = plt
        self._live = live
        self._output_dir = output_dir
        self._header = header          # first title line (clip id, ego shift)
        self._radar_colors: dict[str, tuple] = {}   # radar_id -> stable color
        # Optional (logged, shifted-or-None) ego trajectories, Nx3 world points.
        self._ego_trajectories: tuple[np.ndarray, np.ndarray | None] | None = None
        self._fig, self._ax = plt.subplots(figsize=(10, 10))
        mgr = self._fig.canvas.manager
        if mgr is not None:
            mgr.set_window_title("radar reprojection")
        if live:
            plt.ion()
            self._fig.show()

    def set_ego_trajectories(self, logged_world: np.ndarray,
                             shifted_world: np.ndarray | None = None) -> None:
        """Overlay ego trajectories (Nx3 world points; shifted one optional)."""
        self._ego_trajectories = (logged_world, shifted_world)

    def _color_for(self, radar_id: str) -> tuple:
        color = self._radar_colors.get(radar_id)
        if color is None:
            cmap = self._plt.get_cmap("tab10")
            color = cmap(len(self._radar_colors) % cmap.N)
            self._radar_colors[radar_id] = color
        return color

    def render(self, req: pb.ReprojectRequest, resp: pb.ReprojectResponse,
               scan_index: int) -> None:
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
        if self._ego_trajectories is not None:
            logged_w, shifted_w = self._ego_trajectories
            series = [(logged_w, "black", "--", "ego traj (logged)")]
            if shifted_w is not None:
                series.append((shifted_w, "magenta", "-", "ego traj (shifted)"))
            for pts_w, traj_color, ls, label in series:
                xy = _world_to_ego_xy(req.ego_state.from_world, pts_w)
                ax.plot(xy[:, 0], xy[:, 1], color=traj_color, linestyle=ls,
                        linewidth=1.2, alpha=0.8, zorder=2)
                handles.append(Line2D([], [], color=traj_color, linestyle=ls,
                                      linewidth=1.2))
                labels.append(label)
        handles.append(Line2D([], [], marker="o", linestyle="",
                              color=color, markersize=4))
        labels.append(f"{req.radar_param.id} ({len(xs)})")
        ax.legend(handles=handles, labels=labels, loc="upper right",
                  fontsize="small")
        title = f"{req.radar_param.id}  t={req.timestamp_us} us"
        if self._header:
            title = f"{self._header}\n{title}"
        ax.set_title(title, fontsize=10)

        if self._output_dir is not None:
            png = self._output_dir / req.radar_param.id / f"scan_{scan_index:05d}.png"
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


def _make_dummy_reprojector(log_data: pb.LogData):
    """Return a stub.Reproject stand-in that echoes the original logged scan.

    Requests are generated from the logged scans themselves, so the exact
    (radar id, timestamp) scan always exists. The response uses the regular
    ReprojectResponse shape, so plotting and summaries work unchanged.
    """
    lookup = {
        (radar_id, scan.timestamp_us): scan
        for radar_id, rec in log_data.log_radar_data.items()
        for scan in rec.scans
    }

    def reproject(req: pb.ReprojectRequest) -> pb.ReprojectResponse:
        scan = lookup.get((req.radar_param.id, req.timestamp_us))
        if scan is None:
            raise KeyError(f"no logged scan for {req.radar_param.id} "
                           f"at t={req.timestamp_us}us")
        resp = pb.ReprojectResponse()
        resp.scan.CopyFrom(scan)
        return resp

    return reproject


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
    p.add_argument("--ego-shift", nargs=3, type=float, default=None,
                   metavar=("LAT_M", "LON_M", "HEADING_DEG"),
                   help="Request reprojection from a pose offset from the "
                        "logged ego pose, given in the ego frame: lateral "
                        "metres (+left), longitudinal metres (+forward), "
                        "heading degrees (+counter-clockwise). A pure pose "
                        "shift: the logged world-frame ego velocity is kept, "
                        "and actors stay on their logged trajectories. When "
                        "plotting, the logged and shifted ego trajectories "
                        "are overlaid on every frame.")
    p.add_argument("--dummy", action="store_true",
                   help="Dummy reprojection: skip the server entirely and "
                        "answer each request with the exact original logged "
                        "scan for that radar and timestamp, in the regular "
                        "response format. All downstream functionality "
                        "(plotting, summaries) works unchanged; useful as an "
                        "identity baseline and to visualize the raw log.")
    p.add_argument("--plot-output-dir", type=pathlib.Path, default=None,
                   help="If set, save each frame as "
                        "<dir>/<radar_id>/scan_<index>.png.")
    p.add_argument("--show", action="store_true",
                   help="Show an interactive top-down plot that updates on the "
                        "fly as each radar scan is reprojected. Can be "
                        "combined with --plot-output-dir.")
    p.add_argument("--plot-every", type=int, default=1,
                   help="Plot every N-th scan (default 1). Used when "
                        "--plot-output-dir and/or --show is set.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    dataset_root = _resolve_workspace_path(args.dataset_root)
    intrinsics_config = _resolve_workspace_path(args.intrinsics_config)
    plot_dir = (_resolve_workspace_path(args.plot_output_dir)
                if args.plot_output_dir is not None else None)
    if args.plot_every <= 0:
        print("--plot-every must be a positive integer", file=sys.stderr)
        return 1
    title_header = f"clip {args.clip}"
    if args.dummy:
        title_header += "   dummy: original scans"
    if args.ego_shift is not None:
        s_lat, s_lon, s_hdg = args.ego_shift
        title_header += (f"   ego-shift: lat {s_lat:+g} m, "
                         f"lon {s_lon:+g} m, heading {s_hdg:+g} deg")
    viewer = (_FrameViewer(live=args.show, output_dir=plot_dir,
                           header=title_header)
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
    unknown_radars = sorted(set(args.radars or []) - set(radar_ids))
    if unknown_radars:
        print(f"Unknown radar id(s) for clip {args.clip}: {', '.join(unknown_radars)}",
              file=sys.stderr)
        return 1
    n_ego = len(log_data.log_actor_data.ego_trajectory.states)
    n_radars = len(radar_ids)
    n_scans = sum(len(rec.scans) for rec in log_data.log_radar_data.values())
    print(f"Loaded: {n_ego} ego states, {n_radars} radars, {n_scans} scans, "
          f"{len(log_data.log_actor_data.actor_to_trajectory)} actors", flush=True)
    print(f"Radars: {', '.join(radar_ids)}", flush=True)

    ego_shift = None
    if args.ego_shift is not None:
        if args.dummy:
            print("warning: --dummy echoes the original scans; --ego-shift "
                  "only moves the plot frame, so detections will not match "
                  "the shifted pose.", file=sys.stderr)
        lat_m, lon_m, heading_deg = args.ego_shift
        ego_shift = ri.make_planar_shift_pose(lat_m, lon_m,
                                              math.radians(heading_deg))
        print(f"Ego shift: lateral {lat_m:+g} m (left+), "
              f"longitudinal {lon_m:+g} m (forward+), "
              f"heading {heading_deg:+g} deg (CCW+)", flush=True)

    if viewer is not None and (ego_shift is not None or args.dummy):
        # Overlay the logged (and, with --ego-shift, the shifted) ego
        # trajectory on every frame.
        ego_states = sorted(log_data.log_actor_data.ego_trajectory.states,
                            key=lambda s: s.timestamp_us)
        logged_w = np.array([[s.to_world.translation.x,
                              s.to_world.translation.y,
                              s.to_world.translation.z]
                             for s in ego_states])
        shifted_w = None
        if ego_shift is not None:
            shifted_w = np.array([[p.translation.x, p.translation.y,
                                   p.translation.z]
                                  for p in (ri.compose_poses(s.to_world, ego_shift)
                                            for s in ego_states)])
        viewer.set_ego_trajectories(logged_w, shifted_w)

    total_dets = 0
    i_scan = 0

    with contextlib.ExitStack() as stack:
        if args.dummy:
            print("Dummy mode: echoing original logged scans; no server used.",
                  flush=True)
            reproject = _make_dummy_reprojector(log_data)
        else:
            ch = stack.enter_context(
                grpc.insecure_channel(args.address, options=GRPC_OPTS))
            stub = pb_grpc.RadarReprojectorServiceStub(ch)
            init_req = pb.InitializeRequest(
                param=adapter_obj.reprojector_param(),
                log_data=log_data,
            )
            try:
                stub.Initialize(init_req)
            except grpc.RpcError as e:
                print(f"Initialize failed: {e.code().name} {e.details()}",
                      file=sys.stderr)
                return 1
            reproject = stub.Reproject

        for req in ri.iter_reproject_requests(log_data, ego_shift=ego_shift):
            i_scan += 1
            try:
                resp = reproject(req)
            except grpc.RpcError as e:
                print(f"Reproject failed at scan {i_scan}/{n_scans} "
                      f"radar={req.radar_param.id}: {e.code().name} {e.details()}",
                      file=sys.stderr)
                return 1
            n = len(resp.scan.detections)
            print(f"[scan {i_scan}/{n_scans}  t={req.timestamp_us}us] "
                  f"{req.radar_param.id} -> {n} detections")

            if viewer is not None and (i_scan - 1) % args.plot_every == 0:
                viewer.render(req, resp, i_scan)

            total_dets += n

    skipped = n_scans - i_scan
    print(f"Done. {i_scan} logged scans reprojected across {n_radars} radars, "
          f"{total_dets} detections total."
          + (f" ({skipped} scans outside the ego trajectory window skipped.)"
             if skipped else ""))

    if viewer is not None:
        # Block on the final live frame (if any) until the window is closed.
        viewer.close(block=args.show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
