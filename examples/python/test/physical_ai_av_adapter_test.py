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

"""Integration tests for PhysicalAIAVAdapter against sample_data/."""

import os
import pathlib
import unittest

from examples.python import physical_ai_av_adapter as paa
from proto import radar_reprojector_pb2 as pb


SAMPLE_CLIP = "25cd4769-5dcf-4b53-a351-bf2c5deb6124"


def _sample_data_root() -> pathlib.Path:
    srcdir = os.environ.get("TEST_SRCDIR")
    if srcdir:
        return pathlib.Path(srcdir) / "_main" / "sample_data"
    raise RuntimeError("Run via 'bazel test'; TEST_SRCDIR not set.")


def _intrinsics_config_path() -> pathlib.Path:
    srcdir = os.environ.get("TEST_SRCDIR")
    if srcdir:
        return pathlib.Path(srcdir) / "_main" / "config" / "radar_intrinsics.json"
    raise RuntimeError("Run via 'bazel test'; TEST_SRCDIR not set.")


def _make_adapter():
    return paa.PhysicalAIAVAdapter(
        dataset_root=_sample_data_root(),
        clip_uuid=SAMPLE_CLIP,
        intrinsics_config=_intrinsics_config_path(),
    )


class CalibrationLoadersTest(unittest.TestCase):
    def test_extrinsics_returns_pose_per_radar(self):
        adapter_obj = _make_adapter()
        ext = adapter_obj._load_extrinsics()
        self.assertIn("radar_front_center_srr_0", ext)
        pose_msg = ext["radar_front_center_srr_0"]
        # Translation should be a non-zero vector for a real radar mount.
        norm_sq = (pose_msg.translation.x ** 2 +
                   pose_msg.translation.y ** 2 +
                   pose_msg.translation.z ** 2)
        self.assertGreater(norm_sq, 0.0)
        # Quaternion must be unit-norm.
        q = pose_msg.rotation
        qnorm = (q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2) ** 0.5
        self.assertAlmostEqual(qnorm, 1.0, places=4)

    def test_vehicle_dims_returns_positive_size(self):
        adapter_obj = _make_adapter()
        dims = adapter_obj._load_vehicle_dims()
        self.assertGreater(dims["length"], 0.0)
        self.assertGreater(dims["width"], 0.0)
        self.assertGreater(dims["height"], 0.0)


class EgoTrajectoryTest(unittest.TestCase):
    def test_ego_trajectory_has_many_states_in_order(self):
        adapter_obj = _make_adapter()
        bbox_half = pb.Vec3f(x=2.5, y=1.0, z=0.75)  # placeholder — real test below
        traj = adapter_obj._build_ego_trajectory(bbox_half)
        self.assertGreater(len(traj.states), 100)
        ts_list = [s.timestamp_us for s in traj.states]
        self.assertEqual(ts_list, sorted(ts_list))
        # Velocity from parquet should be non-trivially populated for at least one frame.
        any_nonzero = any(
            (s.velocity_mps.x ** 2 + s.velocity_mps.y ** 2 + s.velocity_mps.z ** 2) > 1e-6
            for s in traj.states
        )
        self.assertTrue(any_nonzero, "ego velocity should have some non-zero entries")
        # bbox_half_size should match the value we passed in.
        for s in traj.states:
            self.assertAlmostEqual(s.bbox_half_size_m.x, 2.5, places=5)
            self.assertAlmostEqual(s.bbox_half_size_m.y, 1.0, places=5)
            self.assertAlmostEqual(s.bbox_half_size_m.z, 0.75, places=5)


class ActorTrajectoryTest(unittest.TestCase):
    def test_actor_trajectories_present_with_finite_diff_velocity(self):
        adapter_obj = _make_adapter()
        bbox_half = pb.Vec3f(x=2.5, y=1.0, z=0.75)
        ego = adapter_obj._build_ego_trajectory(bbox_half)
        actors = adapter_obj._build_actor_trajectories(ego)
        self.assertGreater(len(actors), 0)
        # At least one actor should have multiple states and non-zero velocity.
        any_with_velocity = False
        for traj in actors.values():
            if len(traj.states) >= 2:
                v = traj.states[1].velocity_mps
                if (v.x*v.x + v.y*v.y + v.z*v.z) > 1e-6:
                    any_with_velocity = True
                    break
        self.assertTrue(any_with_velocity, "expect at least one moving actor")
        # Sanity: trajectories are sorted by timestamp.
        for traj in actors.values():
            ts_list = [s.timestamp_us for s in traj.states]
            self.assertEqual(ts_list, sorted(ts_list))


class RadarScanBuilderTest(unittest.TestCase):
    def test_radar_scans_grouped_by_scan_index(self):
        adapter_obj = _make_adapter()
        radar_id = "radar_front_center_srr_0"
        # Build a placeholder RadarParam with an arbitrary id; only the .id is
        # consumed by _build_radar_recording for the reference field.
        param = pb.RadarParam()
        param.id = radar_id
        rec = adapter_obj._build_radar_recording(radar_id, param)
        self.assertGreater(len(rec.scans), 50)
        # Each scan has at least one detection in real data.
        self.assertTrue(all(len(scan.detections) > 0 for scan in rec.scans))
        # First detection's reference should round-trip to the same radar_id.
        first_det = rec.scans[0].detections[0]
        self.assertEqual(first_det.reference.radar_id, radar_id)
        self.assertEqual(first_det.reference.scan_index, 0)
        self.assertEqual(first_det.reference.detection_index, 0)


class LoadLogDataTest(unittest.TestCase):
    def test_load_log_data_full_clip(self):
        adapter_obj = _make_adapter()
        log = adapter_obj.load_log_data()
        self.assertEqual(len(log.log_radar_data), 9)
        self.assertIn("radar_front_center_srr_0", log.log_radar_data)
        rec = log.log_radar_data["radar_front_center_srr_0"]
        # max_num_returns must equal the largest scan size we observed.
        self.assertGreater(rec.param.max_num_returns, 0)
        observed_max = max(len(scan.detections) for scan in rec.scans)
        self.assertEqual(rec.param.max_num_returns, observed_max)
        # Intrinsics must be looked up from the JSON config (srr_0 row).
        self.assertGreater(rec.param.intrinsics.max_range_m, 0.0)
        # Ego + actors populated.
        self.assertGreater(len(log.log_actor_data.ego_trajectory.states), 100)
        self.assertGreater(len(log.log_actor_data.actor_to_trajectory), 0)

    def test_load_log_data_caches(self):
        adapter_obj = _make_adapter()
        a = adapter_obj.load_log_data()
        b = adapter_obj.load_log_data()
        self.assertIs(a, b)


if __name__ == "__main__":
    unittest.main()
