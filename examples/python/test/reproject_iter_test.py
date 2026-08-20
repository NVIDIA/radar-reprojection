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

"""Unit tests for examples/python/reproject_iter.py."""

import math
import unittest

from examples.python import reproject_iter as ri
from proto import radar_reprojector_pb2 as pb


def quat(x, y, z, w):
    return pb.Quaternionf(x=x, y=y, z=z, w=w)


def vec(x, y, z):
    return pb.Vec3f(x=x, y=y, z=z)


def pose(qx, qy, qz, qw, tx, ty, tz):
    p = pb.Pose()
    p.rotation.CopyFrom(quat(qx, qy, qz, qw))
    p.translation.CopyFrom(vec(tx, ty, tz))
    return p


class SlerpQuatTest(unittest.TestCase):
    def test_endpoints(self):
        a = quat(0.0, 0.0, 0.0, 1.0)
        b = quat(0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))
        out0 = ri.slerp_quat(a, b, 0.0)
        out1 = ri.slerp_quat(a, b, 1.0)
        self.assertAlmostEqual(out0.w, 1.0, places=6)
        self.assertAlmostEqual(out1.z, math.sin(math.pi/4), places=6)

    def test_midpoint_is_unit(self):
        a = quat(0.0, 0.0, 0.0, 1.0)
        b = quat(0.0, 0.0, math.sin(math.pi/4), math.cos(math.pi/4))
        out = ri.slerp_quat(a, b, 0.5)
        norm = math.sqrt(out.x**2 + out.y**2 + out.z**2 + out.w**2)
        self.assertAlmostEqual(norm, 1.0, places=6)

    def test_handles_negative_dot(self):
        # Two quats representing the same rotation but opposite signs.
        a = quat(0.0, 0.0, 0.0, 1.0)
        b = quat(0.0, 0.0, 0.0, -1.0)
        out = ri.slerp_quat(a, b, 0.5)
        # Should not blow up; should still be unit-norm.
        norm = math.sqrt(out.x**2 + out.y**2 + out.z**2 + out.w**2)
        self.assertAlmostEqual(norm, 1.0, places=6)


class LerpVec3Test(unittest.TestCase):
    def test_midpoint(self):
        out = ri.lerp_vec3(vec(0, 0, 0), vec(2, 4, 6), 0.5)
        self.assertEqual((out.x, out.y, out.z), (1.0, 2.0, 3.0))


class ComposePosesTest(unittest.TestCase):
    def test_identity_is_neutral(self):
        identity = pose(0, 0, 0, 1, 0, 0, 0)
        p = pose(0, 0, 0, 1, 1.0, 2.0, 3.0)
        out = ri.compose_poses(identity, p)
        self.assertAlmostEqual(out.translation.x, 1.0, places=6)
        self.assertAlmostEqual(out.translation.y, 2.0, places=6)
        self.assertAlmostEqual(out.translation.z, 3.0, places=6)

    def test_translation_only_compose(self):
        a = pose(0, 0, 0, 1, 1, 0, 0)
        b = pose(0, 0, 0, 1, 0, 2, 0)
        out = ri.compose_poses(a, b)
        self.assertAlmostEqual(out.translation.x, 1.0, places=6)
        self.assertAlmostEqual(out.translation.y, 2.0, places=6)

    def test_rotation_rotates_inner_translation(self):
        # 90-degree yaw on the outer pose: a translation of (1,0,0) in the
        # inner frame should land at (0,1,0) in the outer/world frame.
        import math as _math
        yaw = _math.pi / 2
        outer = pose(0.0, 0.0, _math.sin(yaw / 2), _math.cos(yaw / 2), 0.0, 0.0, 0.0)
        inner = pose(0, 0, 0, 1, 1.0, 0.0, 0.0)
        out = ri.compose_poses(outer, inner)
        self.assertAlmostEqual(out.translation.x, 0.0, places=5)
        self.assertAlmostEqual(out.translation.y, 1.0, places=5)
        self.assertAlmostEqual(out.translation.z, 0.0, places=5)


class InvertPoseTest(unittest.TestCase):
    def test_compose_with_inverse_is_identity(self):
        # 90-degree yaw + translation
        p = pose(0, 0, math.sin(math.pi/4), math.cos(math.pi/4), 1.0, 2.0, 3.0)
        inv = ri.invert_pose(p)
        out = ri.compose_poses(p, inv)
        self.assertAlmostEqual(out.translation.x, 0.0, places=5)
        self.assertAlmostEqual(out.translation.y, 0.0, places=5)
        self.assertAlmostEqual(out.translation.z, 0.0, places=5)
        self.assertAlmostEqual(out.rotation.w, 1.0, places=5)


class SlerpActorStateTest(unittest.TestCase):
    def _state(self, t_us, tx, ty, tz):
        s = pb.ActorState()
        s.timestamp_us = t_us
        s.to_world.CopyFrom(pose(0, 0, 0, 1, tx, ty, tz))
        s.from_world.CopyFrom(pose(0, 0, 0, 1, -tx, -ty, -tz))
        s.velocity_mps.CopyFrom(vec(0, 0, 0))
        s.bbox_half_size_m.CopyFrom(vec(1, 1, 1))
        s.bbox_center_m.CopyFrom(vec(0, 0, 0))
        return s

    def test_returns_none_outside_range(self):
        traj = pb.ActorTrajectory()
        traj.states.append(self._state(0, 0, 0, 0))
        traj.states.append(self._state(1_000_000, 10, 0, 0))
        self.assertIsNone(ri.slerp_actor_state(traj, -1))
        self.assertIsNone(ri.slerp_actor_state(traj, 2_000_000))

    def test_interpolates_translation_at_midpoint(self):
        traj = pb.ActorTrajectory()
        traj.states.append(self._state(0, 0, 0, 0))
        traj.states.append(self._state(1_000_000, 10, 0, 0))
        out = ri.slerp_actor_state(traj, 500_000)
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out.to_world.translation.x, 5.0, places=5)
        self.assertEqual(out.timestamp_us, 500_000)

    def test_endpoint_returns_endpoint_state(self):
        traj = pb.ActorTrajectory()
        traj.states.append(self._state(0, 0, 0, 0))
        traj.states.append(self._state(1_000_000, 10, 0, 0))
        out = ri.slerp_actor_state(traj, 1_000_000)
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out.to_world.translation.x, 10.0, places=5)


class IterReprojectRequestsTest(unittest.TestCase):
    def _state(self, t_us, tx):
        s = pb.ActorState()
        s.timestamp_us = t_us
        s.to_world.CopyFrom(pose(0, 0, 0, 1, tx, 0, 0))
        s.from_world.CopyFrom(pose(0, 0, 0, 1, -tx, 0, 0))
        s.velocity_mps.CopyFrom(vec(0, 0, 0))
        s.bbox_half_size_m.CopyFrom(vec(1, 1, 1))
        s.bbox_center_m.CopyFrom(vec(0, 0, 0))
        return s

    def _radar_param(self, radar_id):
        p = pb.RadarParam()
        p.id = radar_id
        p.max_num_returns = 8
        p.intrinsics.min_range_m = 0.1
        p.intrinsics.max_range_m = 100.0
        p.intrinsics.half_hfov_rad = 1.0
        p.intrinsics.half_vfov_rad = 0.5
        p.intrinsics.hfov_res_rad = 0.01
        p.intrinsics.vfov_res_rad = 0.01
        p.intrinsics.range_res_m = 0.1
        p.to_ego.CopyFrom(pose(0, 0, 0, 1, 0, 0, 0))
        return p

    def test_emits_one_request_per_ego_state_per_radar(self):
        log = pb.LogData()
        for radar_id in ["rB", "rA"]:                         # insertion order
            rec = log.log_radar_data[radar_id]
            rec.param.CopyFrom(self._radar_param(radar_id))
        for ts in [0, 100_000, 200_000]:
            log.log_actor_data.ego_trajectory.states.append(self._state(ts, ts * 1e-5))

        reqs = list(ri.iter_reproject_requests(log))
        self.assertEqual(len(reqs), 3 * 2)
        # Iteration order: ego ascending, radar id sorted alphabetically.
        radar_ids = [r.radar_param.id for r in reqs]
        self.assertEqual(radar_ids, ["rA", "rB", "rA", "rB", "rA", "rB"])
        ts_seen = [r.timestamp_us for r in reqs]
        self.assertEqual(ts_seen, [0, 0, 100_000, 100_000, 200_000, 200_000])

    def test_includes_only_actors_in_range(self):
        log = pb.LogData()
        rec = log.log_radar_data["r0"]
        rec.param.CopyFrom(self._radar_param("r0"))
        for ts in [0, 100_000]:
            log.log_actor_data.ego_trajectory.states.append(self._state(ts, 0))
        # Actor "in_range" covers the full ego window.
        in_range = log.log_actor_data.actor_to_trajectory["in_range"]
        in_range.states.append(self._state(0, 5))
        in_range.states.append(self._state(200_000, 7))
        # Actor "too_late" starts after the last ego ts.
        too_late = log.log_actor_data.actor_to_trajectory["too_late"]
        too_late.states.append(self._state(200_000, 0))
        too_late.states.append(self._state(300_000, 10))

        reqs = list(ri.iter_reproject_requests(log))
        self.assertEqual(len(reqs), 2)  # 2 ego states x 1 radar
        for req in reqs:
            self.assertIn("in_range", req.actor_to_state)
            self.assertNotIn("too_late", req.actor_to_state)


if __name__ == "__main__":
    unittest.main()
