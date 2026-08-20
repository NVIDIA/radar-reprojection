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

"""PhysicalAI-AV concrete data adapter.

Mirrors dataset/PhysicalAIAVLoader.cpp: same parquet layout, same column
names, same coordinate conventions. Loads on first load_log_data() call and
caches the result.
"""

from __future__ import annotations

import json
import pathlib
from typing import Optional

import pyarrow.parquet as pq

from examples.python import adapter, reproject_iter as ri
from proto import radar_reprojector_pb2 as pb


class PhysicalAIAVAdapter(adapter.RadarReprojectionDataAdapter):
    def __init__(
        self,
        dataset_root: pathlib.Path,
        clip_uuid: str,
        intrinsics_config: pathlib.Path,
        radar_allowlist: Optional[list[str]] = None,
        reprojector_param: Optional[pb.ReprojectorParam] = None,
    ):
        self._dataset_root = pathlib.Path(dataset_root)
        self._clip_uuid = clip_uuid
        self._intrinsics_config = pathlib.Path(intrinsics_config)
        self._radar_allowlist = radar_allowlist
        self._reprojector_param = (
            reprojector_param
            if reprojector_param is not None
            else adapter.default_reprojector_param()
        )
        self._cached: Optional[pb.LogData] = None

    def reprojector_param(self) -> pb.ReprojectorParam:
        out = pb.ReprojectorParam()
        out.CopyFrom(self._reprojector_param)
        return out

    def load_log_data(self) -> pb.LogData:
        if self._cached is not None:
            return self._cached
        log_data = pb.LogData()

        extrinsics = self._load_extrinsics()
        dims = self._load_vehicle_dims()
        bbox_half = pb.Vec3f(x=0.5 * dims["length"],
                             y=0.5 * dims["width"],
                             z=0.5 * dims["height"])

        log_data.log_actor_data.ego_trajectory.CopyFrom(
            self._build_ego_trajectory(bbox_half))
        for actor_id, traj in self._build_actor_trajectories(
                log_data.log_actor_data.ego_trajectory).items():
            log_data.log_actor_data.actor_to_trajectory[actor_id].CopyFrom(traj)

        intr_table = self._load_intrinsics_config()
        for radar_id in self._radar_dirs():
            param = self._make_radar_param(radar_id, extrinsics, intr_table)
            rec = self._build_radar_recording(radar_id, param)
            param.max_num_returns = max(
                (len(scan.detections) for scan in rec.scans), default=0)
            rec.param.CopyFrom(param)
            log_data.log_radar_data[radar_id].CopyFrom(rec)

        self._cached = log_data
        return log_data

    # --- helpers used by the per-feature loaders in later tasks ------------

    def _radar_dirs(self) -> list[str]:
        radar_root = self._dataset_root / "radar"
        if not radar_root.exists():
            return []
        radars: list[str] = []
        for entry in sorted(radar_root.iterdir()):
            if not entry.is_dir():
                continue
            radar_id = entry.name
            f = entry / f"{self._clip_uuid}.{radar_id}.parquet"
            if f.exists():
                if self._radar_allowlist and radar_id not in self._radar_allowlist:
                    continue
                radars.append(radar_id)
        return radars

    def _find_unique_parquet(self, subdir: pathlib.Path) -> pathlib.Path:
        if not subdir.exists():
            raise FileNotFoundError(f"PhysicalAIAVAdapter: directory missing: {subdir}")
        for entry in sorted(subdir.iterdir()):
            if entry.suffix == ".parquet":
                return entry
        raise FileNotFoundError(f"PhysicalAIAVAdapter: no .parquet under {subdir}")

    def _load_intrinsics_config(self) -> dict[str, dict]:
        with open(self._intrinsics_config, "r") as f:
            return json.load(f)

    @staticmethod
    def _lookup_intrinsics(radar_id: str, intrinsics_table: dict[str, dict]) -> dict:
        """Longest-suffix match against the intrinsics_table keys.

        Mirrors RadarIntrinsicsConfig::lookup in dataset/RadarIntrinsicsConfig.cpp:
        try every key, pick the longest one whose value is a suffix of radar_id.
        Fall back to "srr_0" if no key matches.
        """
        best_key: Optional[str] = None
        for key in intrinsics_table:
            if radar_id.endswith(key) and (best_key is None or len(key) > len(best_key)):
                best_key = key
        if best_key is None:
            if "srr_0" in intrinsics_table:
                return intrinsics_table["srr_0"]
            raise KeyError(
                f"radar_intrinsics.json has no suffix match for radar id '{radar_id}' "
                f"and no 'srr_0' fallback configured.")
        return intrinsics_table[best_key]

    def _load_extrinsics(self) -> dict[str, pb.Pose]:
        path = self._find_unique_parquet(self._dataset_root / "calibration" / "sensor_extrinsics")
        table = pq.read_table(str(path))
        clip_col   = table.column("clip_id").to_pylist()
        sensor_col = table.column("sensor_name").to_pylist()
        qx = table.column("qx").to_pylist(); qy = table.column("qy").to_pylist()
        qz = table.column("qz").to_pylist(); qw = table.column("qw").to_pylist()
        x  = table.column("x").to_pylist();  y  = table.column("y").to_pylist()
        z  = table.column("z").to_pylist()
        out: dict[str, pb.Pose] = {}
        for i, cid in enumerate(clip_col):
            if cid != self._clip_uuid:
                continue
            p = pb.Pose()
            p.translation.x = float(x[i]); p.translation.y = float(y[i]); p.translation.z = float(z[i])
            # Normalize the quaternion to be defensive against minor float drift.
            qx_i, qy_i, qz_i, qw_i = float(qx[i]), float(qy[i]), float(qz[i]), float(qw[i])
            norm = (qx_i*qx_i + qy_i*qy_i + qz_i*qz_i + qw_i*qw_i) ** 0.5 or 1.0
            p.rotation.x = qx_i / norm; p.rotation.y = qy_i / norm
            p.rotation.z = qz_i / norm; p.rotation.w = qw_i / norm
            out[sensor_col[i]] = p
        if not out:
            raise RuntimeError(f"No extrinsics rows for clip {self._clip_uuid} in {path}")
        return out

    def _load_vehicle_dims(self) -> dict[str, float]:
        path = self._find_unique_parquet(self._dataset_root / "calibration" / "vehicle_dimensions")
        table = pq.read_table(str(path))
        clip_col = table.column("clip_id").to_pylist()
        for i, cid in enumerate(clip_col):
            if cid != self._clip_uuid:
                continue
            return {
                "length": float(table.column("length")[i].as_py()),
                "width":  float(table.column("width")[i].as_py()),
                "height": float(table.column("height")[i].as_py()),
            }
        raise RuntimeError(f"No vehicle_dimensions row for clip {self._clip_uuid} in {path}")

    def _build_ego_trajectory(self, bbox_half: pb.Vec3f) -> pb.ActorTrajectory:
        path = (self._dataset_root / "labels" / "egomotion" /
                f"{self._clip_uuid}.egomotion.parquet")
        table = pq.read_table(str(path))
        t  = table.column("timestamp").to_pylist()
        qx = table.column("qx").to_pylist(); qy = table.column("qy").to_pylist()
        qz = table.column("qz").to_pylist(); qw = table.column("qw").to_pylist()
        x  = table.column("x").to_pylist();  y  = table.column("y").to_pylist()
        z  = table.column("z").to_pylist()
        vx = table.column("vx").to_pylist(); vy = table.column("vy").to_pylist()
        vz = table.column("vz").to_pylist()

        rows = []
        for i in range(len(t)):
            rows.append((int(t[i]),
                         float(qx[i]), float(qy[i]), float(qz[i]), float(qw[i]),
                         float(x[i]),  float(y[i]),  float(z[i]),
                         float(vx[i]), float(vy[i]), float(vz[i])))
        rows.sort(key=lambda r: r[0])

        traj = pb.ActorTrajectory()
        for (ts, rx, ry, rz, rw, tx, ty, tz, vxi, vyi, vzi) in rows:
            s = traj.states.add()
            s.timestamp_us = ts
            norm = (rx*rx + ry*ry + rz*rz + rw*rw) ** 0.5 or 1.0
            s.to_world.rotation.x = rx / norm
            s.to_world.rotation.y = ry / norm
            s.to_world.rotation.z = rz / norm
            s.to_world.rotation.w = rw / norm
            s.to_world.translation.x = tx
            s.to_world.translation.y = ty
            s.to_world.translation.z = tz
            s.from_world.CopyFrom(ri.invert_pose(s.to_world))
            s.velocity_mps.x = vxi
            s.velocity_mps.y = vyi
            s.velocity_mps.z = vzi
            s.bbox_half_size_m.CopyFrom(bbox_half)
            # bbox_center_m left zeroed (matches EgoTrajectoryBuilder.cpp:89).
        return traj

    def _build_actor_trajectories(
        self,
        ego_trajectory: pb.ActorTrajectory,
    ) -> dict[str, pb.ActorTrajectory]:
        path = (self._dataset_root / "labels" / "obstacle.offline" /
                f"{self._clip_uuid}.obstacle.offline.parquet")
        table = pq.read_table(str(path))
        ts_col = table.column("timestamp_us").to_pylist()
        id_col = table.column("track_id").to_pylist()
        cx = table.column("center_x").to_pylist(); cy = table.column("center_y").to_pylist()
        cz = table.column("center_z").to_pylist()
        sx = table.column("size_x").to_pylist();   sy = table.column("size_y").to_pylist()
        sz = table.column("size_z").to_pylist()
        ox = table.column("orientation_x").to_pylist(); oy = table.column("orientation_y").to_pylist()
        oz = table.column("orientation_z").to_pylist(); ow = table.column("orientation_w").to_pylist()

        out: dict[str, pb.ActorTrajectory] = {}
        for i in range(len(ts_col)):
            t = int(ts_col[i])
            track_id = id_col[i]
            ego_at_t = ri.slerp_actor_state(ego_trajectory, t)
            if ego_at_t is None:
                # Skip obstacles whose timestamp falls outside the ego window.
                # NOTE: dataset/detail/EgoInterpolator.cpp clamps to the
                # endpoint instead. Python skips by design (matches the
                # iter_reproject_requests skip-on-out-of-range policy and
                # avoids fabricating world-frame poses from extrapolated
                # ego data at clip edges).
                continue
            actor_to_ego = pb.Pose()
            actor_to_ego.translation.x = float(cx[i])
            actor_to_ego.translation.y = float(cy[i])
            actor_to_ego.translation.z = float(cz[i])
            qxi, qyi, qzi, qwi = float(ox[i]), float(oy[i]), float(oz[i]), float(ow[i])
            qnorm = (qxi*qxi + qyi*qyi + qzi*qzi + qwi*qwi) ** 0.5 or 1.0
            actor_to_ego.rotation.x = qxi / qnorm
            actor_to_ego.rotation.y = qyi / qnorm
            actor_to_ego.rotation.z = qzi / qnorm
            actor_to_ego.rotation.w = qwi / qnorm
            actor_to_world = ri.compose_poses(ego_at_t.to_world, actor_to_ego)

            traj = out.setdefault(track_id, pb.ActorTrajectory())
            s = traj.states.add()
            s.timestamp_us = t
            s.to_world.CopyFrom(actor_to_world)
            s.from_world.CopyFrom(ri.invert_pose(actor_to_world))
            # velocity filled below with finite differences.
            s.bbox_half_size_m.x = 0.5 * float(sx[i])
            s.bbox_half_size_m.y = 0.5 * float(sy[i])
            s.bbox_half_size_m.z = 0.5 * float(sz[i])
            # bbox_center_m left zeroed (matches ActorTrajectoryBuilder.cpp:126).

        for traj in out.values():
            traj.states.sort(key=lambda s: s.timestamp_us)
            self._fill_finite_diff_velocity(traj)
        return out

    def _build_radar_recording(
        self,
        radar_id: str,
        param: pb.RadarParam,
    ) -> pb.LogRecording:
        path = (self._dataset_root / "radar" / radar_id /
                f"{self._clip_uuid}.{radar_id}.parquet")
        table = pq.read_table(str(path))
        scan_idx_col = table.column("scan_index").to_pylist()
        ts_col       = table.column("timestamp").to_pylist()
        det_idx_col  = table.column("detection_index").to_pylist()
        az_col       = table.column("azimuth").to_pylist()
        el_col       = table.column("elevation").to_pylist()
        d_col        = table.column("distance").to_pylist()
        rv_col       = table.column("radial_velocity").to_pylist()
        rcs_col      = table.column("rcs").to_pylist()

        rec = pb.LogRecording()
        rec.param.CopyFrom(param)
        cur_scan_idx = -1
        cur_scan: Optional[pb.RadarScan] = None
        for i in range(len(scan_idx_col)):
            si = int(scan_idx_col[i])
            if si != cur_scan_idx:
                cur_scan_idx = si
                cur_scan = rec.scans.add()
                cur_scan.timestamp_us = int(ts_col[i])
            det = cur_scan.detections.add()
            det.azimuth_rad         = float(az_col[i])
            det.elevation_rad       = float(el_col[i])
            det.distance_m          = float(d_col[i])
            det.radial_velocity_mps = float(rv_col[i])
            det.rcs_dbm             = float(rcs_col[i])
            det.reference.radar_id        = radar_id
            det.reference.scan_index      = si
            det.reference.detection_index = int(det_idx_col[i])
            det.reference.actor_id        = ""
            # vel_in_world left zeroed (filled by reproject path on the C++ side).
        return rec

    def _make_radar_param(
        self,
        radar_id: str,
        extrinsics: dict[str, pb.Pose],
        intrinsics_table: dict[str, dict],
    ) -> pb.RadarParam:
        param = pb.RadarParam()
        param.id = radar_id
        intr = self._lookup_intrinsics(radar_id, intrinsics_table)
        param.intrinsics.min_range_m   = float(intr["minRangeM"])
        param.intrinsics.max_range_m   = float(intr["maxRangeM"])
        param.intrinsics.half_hfov_rad = float(intr["halfHfovRad"])
        param.intrinsics.half_vfov_rad = float(intr["halfVfovRad"])
        param.intrinsics.hfov_res_rad  = float(intr["hfovResRad"])
        param.intrinsics.vfov_res_rad  = float(intr["vfovResRad"])
        param.intrinsics.range_res_m   = float(intr["rangeResM"])
        if radar_id in extrinsics:
            param.to_ego.CopyFrom(extrinsics[radar_id])
        else:
            param.to_ego.rotation.w = 1.0  # identity
        param.max_num_returns = 0  # filled in load_log_data after scan build
        return param

    @staticmethod
    def _fill_finite_diff_velocity(traj: pb.ActorTrajectory) -> None:
        n = len(traj.states)
        if n == 0:
            return
        if n == 1:
            traj.states[0].velocity_mps.x = 0.0
            traj.states[0].velocity_mps.y = 0.0
            traj.states[0].velocity_mps.z = 0.0
            return
        def _pos(i):
            t = traj.states[i].to_world.translation
            return (t.x, t.y, t.z)
        def _dt(i, j):
            return max((traj.states[j].timestamp_us - traj.states[i].timestamp_us) * 1e-6, 1e-6)
        # endpoints
        p0, p1 = _pos(0), _pos(1)
        dt01 = _dt(0, 1)
        traj.states[0].velocity_mps.x = (p1[0] - p0[0]) / dt01
        traj.states[0].velocity_mps.y = (p1[1] - p0[1]) / dt01
        traj.states[0].velocity_mps.z = (p1[2] - p0[2]) / dt01
        pe0, pe1 = _pos(n - 2), _pos(n - 1)
        dte = _dt(n - 2, n - 1)
        traj.states[n - 1].velocity_mps.x = (pe1[0] - pe0[0]) / dte
        traj.states[n - 1].velocity_mps.y = (pe1[1] - pe0[1]) / dte
        traj.states[n - 1].velocity_mps.z = (pe1[2] - pe0[2]) / dte
        # interior — central difference. We use `pb_` (with underscore) for
        # the right-bracketing position because `pb` would shadow the proto
        # module imported at the top of the file.
        for i in range(1, n - 1):
            pa, pb_ = _pos(i - 1), _pos(i + 1)
            dti = _dt(i - 1, i + 1)
            traj.states[i].velocity_mps.x = (pb_[0] - pa[0]) / dti
            traj.states[i].velocity_mps.y = (pb_[1] - pa[1]) / dti
            traj.states[i].velocity_mps.z = (pb_[2] - pa[2]) / dti
