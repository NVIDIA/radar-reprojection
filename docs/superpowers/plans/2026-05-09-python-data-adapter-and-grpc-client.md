# Python Data Adapter + gRPC Client Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python harness that drives the radar-reprojection gRPC server end-to-end on a real PhysicalAI-AV clip — with an abstract data-adapter base class users can subclass for their own dataset format.

**Architecture:** A two-method abstract adapter (`load_log_data() → pb.LogData`, `reprojector_param() → pb.ReprojectorParam`), one concrete `PhysicalAIAVAdapter` mirroring `dataset/PhysicalAIAVLoader.cpp`, a shared `iter_reproject_requests` utility that interpolates actor states with hand-rolled slerp/lerp, and a `run_clip.py` CLI that wires them together.

**Tech Stack:** Python 3.10, gRPC (grpcio 1.66.0), Protocol Buffers (5.29.0), pyarrow (parquet I/O), numpy (vector math), Bazel (build + test).

---

## File Structure

| Path                                                | Responsibility                                                                  |
| --------------------------------------------------- | ------------------------------------------------------------------------------- |
| `examples/python/adapter.py`                        | Abstract `RadarReprojectionDataAdapter` + `default_reprojector_param()`.        |
| `examples/python/reproject_iter.py`                 | Pose-math helpers, `slerp_actor_state`, `iter_reproject_requests`.              |
| `examples/python/physical_ai_av_adapter.py`         | Concrete `PhysicalAIAVAdapter` for the PhysicalAI-AV parquet layout.            |
| `examples/python/run_clip.py`                       | CLI client: load clip → Initialize → Reproject for every (ego, radar) pair.    |
| `examples/python/test/reproject_iter_test.py`       | Unit tests for the pose-math helpers and iteration logic.                       |
| `examples/python/test/physical_ai_av_adapter_test.py` | Integration test against `sample_data/`.                                      |
| `examples/python/BUILD.bazel`                       | New `py_library`/`py_binary`/`py_test` targets alongside the existing smoke client. |
| `requirements.txt`                                  | Add `numpy` and `pyarrow` pins.                                                 |
| `README.md`                                         | New "Run a real clip" subsection under the gRPC server quick-start.             |

`smoke_client.py` is unchanged.

---

## Pre-flight

All `bazel` invocations assume `scripts/env.sh` has been sourced in the current shell:

```bash
source scripts/env.sh    # sets PATH and CONDA_PREFIX
```

If you skip this step, `bazel build` will refuse to start with a `CONDA_PREFIX env var is not set` error.

---

## Task 1: Add numpy + pyarrow to requirements.txt

**Files:**
- Modify: `requirements.txt`

- [ ] **Step 1: Edit `requirements.txt`**

Replace contents with:

```
grpcio==1.66.0
protobuf==5.29.0
numpy==1.26.4
pyarrow==15.0.2
```

- [ ] **Step 2: Verify bazel pip resolution still succeeds**

Run:

```bash
bazel build //examples/python:smoke_client
```

Expected: build completes successfully (no errors from `@pip` resolving). If you see an error like `pip_repository: failed to install requirement`, the simple-pin lock format isn't enough for these wheels — fall back to:

```bash
pip install pip-tools
echo -e "grpcio==1.66.0\nprotobuf==5.29.0\nnumpy==1.26.4\npyarrow==15.0.2" > requirements.in
pip-compile --output-file=requirements.txt requirements.in
```

then re-run `bazel build //examples/python:smoke_client`.

- [ ] **Step 3: Commit**

```bash
git add requirements.txt
git commit -m "Add numpy + pyarrow to Python deps for new data adapter"
```

---

## Task 2: Adapter ABC + default reprojector param

**Files:**
- Create: `examples/python/adapter.py`
- Modify: `examples/python/BUILD.bazel`
- Test: (none — no behavior beyond abstract interface; covered indirectly via the concrete adapter test in Task 11)

- [ ] **Step 1: Write `examples/python/adapter.py`**

```python
"""Abstract data adapter for the radar-reprojection gRPC server.

Subclass RadarReprojectionDataAdapter to plug in a new dataset format. The
adapter's job is to convert a clip's logged radar + ego/actor data into the
proto messages the server consumes. Iteration over ego poses for Reproject
calls is handled by reproject_iter.iter_reproject_requests, not by adapters.
"""

from abc import ABC, abstractmethod

from proto import radar_reprojector_pb2 as pb


class RadarReprojectionDataAdapter(ABC):
    @abstractmethod
    def load_log_data(self) -> pb.LogData:
        """Return a LogData proto: per-radar recordings + ego/actor trajectories.

        Implementations should cache the result so repeat calls are cheap.
        """

    @abstractmethod
    def reprojector_param(self) -> pb.ReprojectorParam:
        """Return the algorithm-tuning ReprojectorParam.

        Subclasses that don't need to tune the algorithm may simply return
        default_reprojector_param().
        """


def default_reprojector_param() -> pb.ReprojectorParam:
    """ReprojectorParam with conservative defaults safe for any clip."""
    p = pb.ReprojectorParam()
    p.use_log_intrinsics = True
    p.do_down_sampling = False
    p.max_search_size = 30
    p.dump = False
    p.bbox_scale_factor.x = 1.0
    p.bbox_scale_factor.y = 1.0
    p.bbox_scale_factor.z = 1.0
    p.bbox_margin.x = 0.0
    p.bbox_margin.y = 0.0
    p.bbox_margin.z = 0.0
    return p
```

- [ ] **Step 2: Update `examples/python/BUILD.bazel`**

Add a `load("@rules_python//python:defs.bzl", "py_library", "py_test")` entry alongside the existing `py_binary` import (the load line should already include `py_binary` — extend it). Then append after the `smoke_client` target:

```python
py_library(
    name = "adapter",
    srcs = ["adapter.py"],
    deps = ["//proto:radar_reprojector_py_proto"],
    visibility = ["//visibility:public"],
)
```

The full top-of-file load line should read:

```python
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("@pip//:requirements.bzl", "requirement")
```

- [ ] **Step 3: Verify the library builds**

Run:

```bash
bazel build //examples/python:adapter
```

Expected: build completes successfully.

- [ ] **Step 4: Commit**

```bash
git add examples/python/adapter.py examples/python/BUILD.bazel
git commit -m "Adapter ABC + default ReprojectorParam"
```

---

## Task 3: Pose-math helpers in reproject_iter.py

**Files:**
- Create: `examples/python/reproject_iter.py`
- Create: `examples/python/test/reproject_iter_test.py`
- Modify: `examples/python/BUILD.bazel`

- [ ] **Step 1: Write the failing test `examples/python/test/reproject_iter_test.py`**

```python
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


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Add the test target to `examples/python/BUILD.bazel`**

Append:

```python
py_test(
    name = "reproject_iter_test",
    srcs = ["test/reproject_iter_test.py"],
    deps = [
        ":reproject_iter",
        "//proto:radar_reprojector_py_proto",
    ],
    main = "test/reproject_iter_test.py",
)
```

- [ ] **Step 3: Run the test (expect failure — module doesn't exist yet)**

```bash
bazel test //examples/python:reproject_iter_test
```

Expected: build error or test failure citing missing `reproject_iter` module.

- [ ] **Step 4: Implement `examples/python/reproject_iter.py` with pose-math helpers**

```python
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
```

- [ ] **Step 5: Add the `reproject_iter` `py_library` target**

In `examples/python/BUILD.bazel`, add (above the `py_test` target):

```python
py_library(
    name = "reproject_iter",
    srcs = ["reproject_iter.py"],
    deps = [
        "//proto:radar_reprojector_py_proto",
        requirement("numpy"),
    ],
    visibility = ["//visibility:public"],
)
```

- [ ] **Step 6: Run the test (expect PASS)**

```bash
bazel test //examples/python:reproject_iter_test
```

Expected: all 6 cases pass.

- [ ] **Step 7: Commit**

```bash
git add examples/python/reproject_iter.py examples/python/test/reproject_iter_test.py examples/python/BUILD.bazel
git commit -m "Pose-math helpers (slerp, lerp, compose, invert) in reproject_iter"
```

---

## Task 4: slerp_actor_state

**Files:**
- Modify: `examples/python/reproject_iter.py`
- Modify: `examples/python/test/reproject_iter_test.py`

- [ ] **Step 1: Add the failing test cases to `reproject_iter_test.py`**

Append:

```python
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
```

- [ ] **Step 2: Run the test (expect failure)**

```bash
bazel test //examples/python:reproject_iter_test
```

Expected: failure with `module 'reproject_iter' has no attribute 'slerp_actor_state'`.

- [ ] **Step 3: Implement `slerp_actor_state` in `reproject_iter.py`**

Append:

```python
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
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:reproject_iter_test
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add examples/python/reproject_iter.py examples/python/test/reproject_iter_test.py
git commit -m "slerp_actor_state with bracketing + skip-out-of-range"
```

---

## Task 5: iter_reproject_requests

**Files:**
- Modify: `examples/python/reproject_iter.py`
- Modify: `examples/python/test/reproject_iter_test.py`

- [ ] **Step 1: Add the failing test to `reproject_iter_test.py`**

Append:

```python
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
```

- [ ] **Step 2: Run the test (expect failure)**

```bash
bazel test //examples/python:reproject_iter_test
```

Expected: failure citing missing `iter_reproject_requests`.

- [ ] **Step 3: Implement `iter_reproject_requests` in `reproject_iter.py`**

Append:

```python
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
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:reproject_iter_test
```

Expected: all tests pass (8 cases total now).

- [ ] **Step 5: Commit**

```bash
git add examples/python/reproject_iter.py examples/python/test/reproject_iter_test.py
git commit -m "iter_reproject_requests: ego x radar iteration with actor interp"
```

---

## Task 6: PhysicalAIAVAdapter — module skeleton + reprojector_param

**Files:**
- Create: `examples/python/physical_ai_av_adapter.py`
- Modify: `examples/python/BUILD.bazel`

- [ ] **Step 1: Write `examples/python/physical_ai_av_adapter.py`**

```python
"""PhysicalAI-AV concrete data adapter.

Mirrors dataset/PhysicalAIAVLoader.cpp: same parquet layout, same column
names, same coordinate conventions. Loads on first load_log_data() call and
caches the result.
"""

from __future__ import annotations

import json
import pathlib
from typing import Optional

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
        # Population happens in Tasks 7–11; for now we leave the LogData empty.
        # Subsequent tasks will replace this body with real loaders.
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
```

- [ ] **Step 2: Add the `physical_ai_av_adapter` `py_library` target**

In `examples/python/BUILD.bazel`, append:

```python
py_library(
    name = "physical_ai_av_adapter",
    srcs = ["physical_ai_av_adapter.py"],
    deps = [
        ":adapter",
        ":reproject_iter",
        "//proto:radar_reprojector_py_proto",
        requirement("numpy"),
        requirement("pyarrow"),
    ],
    visibility = ["//visibility:public"],
)
```

- [ ] **Step 3: Verify the library builds**

```bash
bazel build //examples/python:physical_ai_av_adapter
```

Expected: build completes successfully.

- [ ] **Step 4: Commit**

```bash
git add examples/python/physical_ai_av_adapter.py examples/python/BUILD.bazel
git commit -m "PhysicalAIAVAdapter skeleton + radar dir discovery"
```

---

## Task 7: PhysicalAIAVAdapter — calibration loaders

**Files:**
- Modify: `examples/python/physical_ai_av_adapter.py`
- Create: `examples/python/test/physical_ai_av_adapter_test.py`
- Modify: `examples/python/BUILD.bazel`

- [ ] **Step 1: Write the failing test**

`examples/python/test/physical_ai_av_adapter_test.py`:

```python
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


if __name__ == "__main__":
    unittest.main()
```

Add the test target to `examples/python/BUILD.bazel`:

```python
py_test(
    name = "physical_ai_av_adapter_test",
    srcs = ["test/physical_ai_av_adapter_test.py"],
    deps = [
        ":physical_ai_av_adapter",
        "//proto:radar_reprojector_py_proto",
    ],
    data = [
        "//sample_data:fixture_clip",
        "//config:radar_intrinsics_json",
    ],
    main = "test/physical_ai_av_adapter_test.py",
)
```

- [ ] **Step 2: Run the test (expect failure)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: failure with `'PhysicalAIAVAdapter' object has no attribute '_load_extrinsics'`.

- [ ] **Step 3: Implement the calibration loaders in `physical_ai_av_adapter.py`**

Add at the top of the file (after the existing imports):

```python
import pyarrow.parquet as pq
```

Append these methods to `PhysicalAIAVAdapter`:

```python
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
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: 2 cases pass.

- [ ] **Step 5: Commit**

```bash
git add examples/python/physical_ai_av_adapter.py examples/python/test/physical_ai_av_adapter_test.py examples/python/BUILD.bazel
git commit -m "PhysicalAIAVAdapter: extrinsics + vehicle-dimensions loaders"
```

---

## Task 8: PhysicalAIAVAdapter — ego trajectory builder

**Files:**
- Modify: `examples/python/physical_ai_av_adapter.py`
- Modify: `examples/python/test/physical_ai_av_adapter_test.py`

- [ ] **Step 1: Add the failing test**

Append to `physical_ai_av_adapter_test.py`:

```python
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
```

- [ ] **Step 2: Run the test (expect failure)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: failure with missing `_build_ego_trajectory`.

- [ ] **Step 3: Implement `_build_ego_trajectory`**

Append to `PhysicalAIAVAdapter` in `physical_ai_av_adapter.py`:

```python
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
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: 3 cases pass.

- [ ] **Step 5: Commit**

```bash
git add examples/python/physical_ai_av_adapter.py examples/python/test/physical_ai_av_adapter_test.py
git commit -m "PhysicalAIAVAdapter: ego trajectory builder"
```

---

## Task 9: PhysicalAIAVAdapter — actor trajectory builder

**Files:**
- Modify: `examples/python/physical_ai_av_adapter.py`
- Modify: `examples/python/test/physical_ai_av_adapter_test.py`

- [ ] **Step 1: Add the failing test**

Append to `physical_ai_av_adapter_test.py`:

```python
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
```

- [ ] **Step 2: Run the test (expect failure)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: failure with missing `_build_actor_trajectories`.

- [ ] **Step 3: Implement `_build_actor_trajectories`**

Append to `PhysicalAIAVAdapter`:

```python
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
                continue   # obstacle outside ego window — skip
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
        # interior — central difference
        for i in range(1, n - 1):
            pa, pb_ = _pos(i - 1), _pos(i + 1)
            dti = _dt(i - 1, i + 1)
            traj.states[i].velocity_mps.x = (pb_[0] - pa[0]) / dti
            traj.states[i].velocity_mps.y = (pb_[1] - pa[1]) / dti
            traj.states[i].velocity_mps.z = (pb_[2] - pa[2]) / dti
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: 4 cases pass.

- [ ] **Step 5: Commit**

```bash
git add examples/python/physical_ai_av_adapter.py examples/python/test/physical_ai_av_adapter_test.py
git commit -m "PhysicalAIAVAdapter: actor trajectory builder + finite-diff velocity"
```

---

## Task 10: PhysicalAIAVAdapter — radar scan builder

**Files:**
- Modify: `examples/python/physical_ai_av_adapter.py`
- Modify: `examples/python/test/physical_ai_av_adapter_test.py`

- [ ] **Step 1: Add the failing test**

Append to `physical_ai_av_adapter_test.py`:

```python
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
```

- [ ] **Step 2: Run the test (expect failure)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: failure with missing `_build_radar_recording`.

- [ ] **Step 3: Implement `_build_radar_recording`**

Append to `PhysicalAIAVAdapter`:

```python
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
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: 5 cases pass.

- [ ] **Step 5: Commit**

```bash
git add examples/python/physical_ai_av_adapter.py examples/python/test/physical_ai_av_adapter_test.py
git commit -m "PhysicalAIAVAdapter: radar scan builder grouped by scan_index"
```

---

## Task 11: Wire up load_log_data + RadarParam composition

**Files:**
- Modify: `examples/python/physical_ai_av_adapter.py`
- Modify: `examples/python/test/physical_ai_av_adapter_test.py`

- [ ] **Step 1: Add the failing top-level test**

Append to `physical_ai_av_adapter_test.py`:

```python
class LoadLogDataTest(unittest.TestCase):
    def test_load_log_data_full_clip(self):
        adapter_obj = _make_adapter()
        log = adapter_obj.load_log_data()
        self.assertEqual(len(log.log_radar_data), 8)
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
```

- [ ] **Step 2: Run the test (expect failure — currently `load_log_data` returns an empty LogData)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: failure (`len(log.log_radar_data) == 0`, not 8).

- [ ] **Step 3: Replace the body of `load_log_data` and add the per-radar param helper**

Replace the existing `load_log_data` method body in `physical_ai_av_adapter.py` with:

```python
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
```

- [ ] **Step 4: Run the test (expect PASS)**

```bash
bazel test //examples/python:physical_ai_av_adapter_test
```

Expected: 7 cases pass.

- [ ] **Step 5: Commit**

```bash
git add examples/python/physical_ai_av_adapter.py examples/python/test/physical_ai_av_adapter_test.py
git commit -m "PhysicalAIAVAdapter: wire up load_log_data + RadarParam composition"
```

---

## Task 12: run_clip.py CLI client

**Files:**
- Create: `examples/python/run_clip.py`
- Modify: `examples/python/BUILD.bazel`

(No automated test — this is end-to-end and needs a running server. We'll smoke it manually.)

- [ ] **Step 1: Write `examples/python/run_clip.py`**

```python
"""Drive the radar-reprojection gRPC server end-to-end on a real clip.

Example:
    bazel run //examples/python:run_clip -- \\
        --address=localhost:50051 \\
        --dataset-root=sample_data \\
        --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \\
        --intrinsics-config=config/radar_intrinsics.json
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import grpc

from examples.python import physical_ai_av_adapter as paa
from examples.python import reproject_iter as ri
from proto import radar_reprojector_pb2 as pb
from proto import radar_reprojector_pb2_grpc as pb_grpc


GRPC_OPTS = [
    ("grpc.max_receive_message_length", 256 * 1024 * 1024),
    ("grpc.max_send_message_length",    256 * 1024 * 1024),
]


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
    return p.parse_args()


def main() -> int:
    args = parse_args()

    adapter_obj = paa.PhysicalAIAVAdapter(
        dataset_root=args.dataset_root,
        clip_uuid=args.clip,
        intrinsics_config=args.intrinsics_config,
        radar_allowlist=args.radars or None,
    )

    print(f"Loading {args.clip} from {args.dataset_root} ...", flush=True)
    log_data = adapter_obj.load_log_data()
    n_ego = len(log_data.log_actor_data.ego_trajectory.states)
    n_radars = len(log_data.log_radar_data)
    print(f"Loaded: {n_ego} ego states, {n_radars} radars, "
          f"{len(log_data.log_actor_data.actor_to_trajectory)} actors", flush=True)

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
            total_dets += n

    print(f"Done. {n_ego} ego poses x {n_radars} radars = {n_ego * n_radars} "
          f"reprojections, {total_dets} detections total.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Add the `run_clip` `py_binary` target to `examples/python/BUILD.bazel`**

```python
py_binary(
    name = "run_clip",
    srcs = ["run_clip.py"],
    deps = [
        ":physical_ai_av_adapter",
        ":reproject_iter",
        "//proto:radar_reprojector_py_proto",
        "//proto:radar_reprojector_py_grpc",
        requirement("grpcio"),
    ],
    main = "run_clip.py",
    visibility = ["//visibility:public"],
)
```

- [ ] **Step 3: Verify the binary builds**

```bash
bazel build //examples/python:run_clip
```

Expected: build completes successfully.

- [ ] **Step 4: Manual end-to-end smoke**

In one terminal:

```bash
source scripts/env.sh
bazel run //server:radar_reprojection_server -- --address=0.0.0.0:50051
```

In another:

```bash
source scripts/env.sh
bazel run //examples/python:run_clip -- \
    --address=localhost:50051 \
    --dataset-root=sample_data \
    --clip=25cd4769-5dcf-4b53-a351-bf2c5deb6124 \
    --intrinsics-config=config/radar_intrinsics.json
```

Expected: per-frame output lines like

```
[ego 12/250  t=1234567us] radar_front_center_srr_0 -> 47 detections
```

ending with a `Done.` summary line and exit code 0. Stop the server with Ctrl-C.

- [ ] **Step 5: Commit**

```bash
git add examples/python/run_clip.py examples/python/BUILD.bazel
git commit -m "run_clip.py: end-to-end CLI client driving Initialize + Reproject"
```

---

## Task 13: README "Run a real clip" subsection

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add a subsection under "Running the gRPC server"**

In `README.md`, after the existing block describing the smoke client, append:

````markdown
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

To plug in a different dataset format, subclass
`examples.python.adapter.RadarReprojectionDataAdapter` and pass your
implementation in place of `PhysicalAIAVAdapter`. Iteration over ego poses
and actor-state interpolation are handled by
`examples.python.reproject_iter.iter_reproject_requests`, which works with
any `pb.LogData`.
````

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "README: document run_clip.py and the data adapter interface"
```

---

## Final verification

- [ ] Run all Python tests:

```bash
bazel test //examples/python:reproject_iter_test //examples/python:physical_ai_av_adapter_test
```

Expected: both pass.

- [ ] Run the full repo test suite to make sure nothing C++ regressed:

```bash
bazel test //...
```

Expected: all green.
