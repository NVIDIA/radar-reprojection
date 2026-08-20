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

"""Iteration utility + pose-math helpers for the gRPC client.

The public entry point is iter_reproject_requests(log_data); the helpers
below are also re-used by physical_ai_av_adapter to convert obstacle poses
from ego frame to world frame.
"""

from typing import Iterator, Optional

import numpy as np

from proto import radar_reprojector_pb2 as pb


# ---------------------------------------------------------------------------
# Quaternion + vector primitives
# ---------------------------------------------------------------------------

def slerp_quat(q1: pb.Quaternionf, q2: pb.Quaternionf, ratio: float) -> pb.Quaternionf:
    a = np.array([q1.x, q1.y, q1.z, q1.w], dtype=np.float64)
    b = np.array([q2.x, q2.y, q2.z, q2.w], dtype=np.float64)
    if np.dot(a, b) < 0.0:
        b = -b
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if dot > 0.9995:
        out = a + ratio * (b - a)
    else:
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        s1 = np.sin((1.0 - ratio) * theta) / sin_theta
        s2 = np.sin(ratio * theta) / sin_theta
        out = s1 * a + s2 * b
    out = out / np.linalg.norm(out)
    return pb.Quaternionf(x=float(out[0]), y=float(out[1]), z=float(out[2]), w=float(out[3]))


def lerp_vec3(v1: pb.Vec3f, v2: pb.Vec3f, ratio: float) -> pb.Vec3f:
    return pb.Vec3f(
        x=v1.x + ratio * (v2.x - v1.x),
        y=v1.y + ratio * (v2.y - v1.y),
        z=v1.z + ratio * (v2.z - v1.z),
    )


def lerp_pose(p1: pb.Pose, p2: pb.Pose, ratio: float) -> pb.Pose:
    out = pb.Pose()
    out.translation.CopyFrom(lerp_vec3(p1.translation, p2.translation, ratio))
    out.rotation.CopyFrom(slerp_quat(p1.rotation, p2.rotation, ratio))
    return out


def quat_to_rot_matrix(q: pb.Quaternionf) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def mul_quat(a: pb.Quaternionf, b: pb.Quaternionf) -> pb.Quaternionf:
    return pb.Quaternionf(
        w=a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        x=a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        y=a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        z=a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
    )


def compose_poses(a: pb.Pose, b: pb.Pose) -> pb.Pose:
    """Return a * b (apply b first, then a)."""
    R_a = quat_to_rot_matrix(a.rotation)
    t_b = np.array([b.translation.x, b.translation.y, b.translation.z], dtype=np.float64)
    new_t = np.array([a.translation.x, a.translation.y, a.translation.z], dtype=np.float64) + R_a @ t_b
    out = pb.Pose()
    out.rotation.CopyFrom(mul_quat(a.rotation, b.rotation))
    out.translation.CopyFrom(pb.Vec3f(x=float(new_t[0]), y=float(new_t[1]), z=float(new_t[2])))
    return out


def invert_pose(p: pb.Pose) -> pb.Pose:
    inv_q = pb.Quaternionf(x=-p.rotation.x, y=-p.rotation.y, z=-p.rotation.z, w=p.rotation.w)
    R_t = quat_to_rot_matrix(inv_q)  # = R^T of original
    t = np.array([p.translation.x, p.translation.y, p.translation.z], dtype=np.float64)
    inv_t = -R_t @ t
    out = pb.Pose()
    out.rotation.CopyFrom(inv_q)
    out.translation.CopyFrom(pb.Vec3f(x=float(inv_t[0]), y=float(inv_t[1]), z=float(inv_t[2])))
    return out


# ---------------------------------------------------------------------------
# Trajectory interpolation
# ---------------------------------------------------------------------------

def slerp_actor_state(
    traj: pb.ActorTrajectory,
    t_us: int,
) -> Optional[pb.ActorState]:
    """Return the ActorState at t_us, interpolating between bracketing samples.

    Returns None if t_us falls outside the trajectory's timestamp range
    (no extrapolation).
    """
    if len(traj.states) == 0:
        return None
    states = traj.states
    if t_us < states[0].timestamp_us or t_us > states[-1].timestamp_us:
        return None

    # Binary search for the upper-bound index.
    lo, hi = 0, len(states) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if states[mid].timestamp_us < t_us:
            lo = mid + 1
        else:
            hi = mid
    j = lo
    if states[j].timestamp_us == t_us or j == 0:
        # Exact hit on a sample, or at the start: return a copy.
        out = pb.ActorState()
        out.CopyFrom(states[j])
        out.timestamp_us = t_us
        return out
    i = j - 1
    s1, s2 = states[i], states[j]
    dt = s2.timestamp_us - s1.timestamp_us
    ratio = (t_us - s1.timestamp_us) / dt if dt > 0 else 0.0

    out = pb.ActorState()
    out.timestamp_us = t_us
    out.to_world.CopyFrom(lerp_pose(s1.to_world, s2.to_world, ratio))
    out.from_world.CopyFrom(invert_pose(out.to_world))
    out.velocity_mps.CopyFrom(lerp_vec3(s1.velocity_mps, s2.velocity_mps, ratio))
    out.bbox_half_size_m.CopyFrom(lerp_vec3(s1.bbox_half_size_m, s2.bbox_half_size_m, ratio))
    out.bbox_center_m.CopyFrom(lerp_vec3(s1.bbox_center_m, s2.bbox_center_m, ratio))
    return out


def iter_reproject_requests(log_data: pb.LogData) -> Iterator[pb.ReprojectRequest]:
    """Yield one ReprojectRequest per (ego_state, radar) pair.

    Iteration order: ego timestamps ascending, radar ids sorted alphabetically.
    Actor states are interpolated to each ego timestamp; actors whose
    trajectory does not bracket the ego timestamp are skipped (no extrapolation).
    """
    ego_states = sorted(
        log_data.log_actor_data.ego_trajectory.states,
        key=lambda s: s.timestamp_us,
    )
    radar_ids = sorted(log_data.log_radar_data.keys())

    for ego in ego_states:
        actor_states_at_t: dict[str, pb.ActorState] = {}
        for actor_id, traj in log_data.log_actor_data.actor_to_trajectory.items():
            interp = slerp_actor_state(traj, ego.timestamp_us)
            if interp is not None:
                actor_states_at_t[actor_id] = interp
        for radar_id in radar_ids:
            req = pb.ReprojectRequest()
            req.timestamp_us = ego.timestamp_us
            req.radar_param.CopyFrom(log_data.log_radar_data[radar_id].param)
            req.ego_state.CopyFrom(ego)
            for k, v in actor_states_at_t.items():
                req.actor_to_state[k].CopyFrom(v)
            yield req
