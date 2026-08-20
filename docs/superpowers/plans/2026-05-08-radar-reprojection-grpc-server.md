# Radar Reprojection gRPC Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up a C++ gRPC server (`RadarReprojectionServer`) that exposes `algorithm::RadarReprojector::initialize()` and `reproject()` over the wire, plus generated Python client stubs and a smoke client. Remove `RadarReprojectionManager`.

**Architecture:** Pure algorithm-wrapper server. Single global `RadarReprojector` guarded by one mutex; `Initialize` may be called repeatedly to swap state. Wire format already authored at `proto/radar_reprojector.proto`. Conversions are free functions in their own translation unit so they can be unit-tested without grpc.

**Tech Stack:** Bazel (bzlmod), C++20, gRPC 1.66, protobuf 29, Eigen, googletest, rules_python + grpcio.

**Spec:** `docs/superpowers/specs/2026-05-08-radar-reprojection-grpc-server-design.md`.

---

## File Structure

| Path | Responsibility |
|---|---|
| `MODULE.bazel` | Add `grpc`, `protobuf`, `rules_proto`, `rules_python` bazel_deps; configure pip extension. |
| `requirements.txt` | Pin `grpcio` + `protobuf` versions for the Python smoke client. |
| `proto/radar_reprojector.proto` | Drop `success` / `error_message` from `InitializeResponse` (rest is unchanged). |
| `proto/BUILD.bazel` | `proto_library`, `cc_proto_library`, `cc_grpc_library`, `py_proto_library`, `py_grpc_library`. |
| `server/Conversions.hpp` | Public free-function declarations for proto↔algorithm. |
| `server/Conversions.cpp` | Implementations. Recomputes `from_world` from `to_world.inverse()`. |
| `server/RadarReprojectionServer.hpp` | Service-impl class declaration. |
| `server/RadarReprojectionServer.cpp` | Service-impl with mutex-guarded state. |
| `server/MainServer.cpp` | CLI flag parsing, `ServerBuilder`, signal-driven shutdown. |
| `server/BUILD.bazel` | `:conversions`, `:radar_reprojection_server_lib`, `:radar_reprojection_server`, two `cc_test`s. |
| `server/test/Conversions_test.cpp` | Round-trip every type pair; `from_world` is recomputed. |
| `server/test/RadarReprojectionServer_test.cpp` | In-process channel; status-code paths. |
| `examples/python/smoke_client.py` | Tiny end-to-end Initialize+Reproject script. |
| `examples/python/BUILD.bazel` | `py_binary` for the smoke client. |
| `RadarReprojectionManager.{hpp,cpp}` | **Deleted.** |
| `BUILD.bazel` (top level) | Drop `radar_reprojection_manager` target. |
| `dataset/test/RadarReprojectionManager_test.cpp` | **Deleted.** |
| `dataset/BUILD.bazel` | Drop the deleted test target. |
| `dataset/RadarScan.hpp` | One stale comment updated. |
| `README.md` | Replace RRM section with server quick-start. |

---

## Task 1: Add gRPC / protobuf deps to MODULE.bazel and update proto

**Files:**
- Modify: `MODULE.bazel`
- Modify: `proto/radar_reprojector.proto`
- Create: `requirements.txt`

- [ ] **Step 1: Edit `MODULE.bazel` to add the new bazel_deps**

Append after the existing `bazel_dep` lines and before the local-repo section:

```python
bazel_dep(name = "grpc",         version = "1.66.0")
bazel_dep(name = "protobuf",     version = "29.0")
bazel_dep(name = "rules_proto",  version = "7.0.2")
bazel_dep(name = "rules_python", version = "0.36.0")

pip = use_extension("@rules_python//python/extensions:pip.bzl", "pip")
pip.parse(
    hub_name        = "pip",
    python_version  = "3.10",
    requirements_lock = "//:requirements.txt",
)
use_repo(pip, "pip")
```

- [ ] **Step 2: Create `requirements.txt`**

```text
grpcio==1.66.0
protobuf==5.28.2
```

- [ ] **Step 3: Drop `success` / `error_message` from `InitializeResponse`**

In `proto/radar_reprojector.proto` replace:

```proto
message InitializeResponse {
    bool success         = 1;
    string error_message = 2; // populated when success == false
}
```

with:

```proto
// Empty payload; errors are reported via gRPC Status (see design §8). The
// message exists as a named type so future fields can be added in a
// wire-compatible way.
message InitializeResponse {
}
```

- [ ] **Step 4: Verify the dep graph resolves**

Run: `bazel mod graph 2>&1 | head -40`

Expected: prints a tree rooted at `radar_reprojection@0.1.0` containing the new deps. **No "version conflict" errors.** If conflicts surface, bump the offending version (most likely `protobuf` or `googletest`) until `bazel mod graph` is clean. The chosen versions above are starting points, not religion.

- [ ] **Step 5: Commit**

```bash
git add MODULE.bazel requirements.txt proto/radar_reprojector.proto
git commit -m "Add gRPC / protobuf bazel_deps and finalize proto API

Drops success/error_message from InitializeResponse; errors travel via
gRPC Status now."
```

---

## Task 2: Write `proto/BUILD.bazel` and verify codegen

**Files:**
- Create: `proto/BUILD.bazel`

- [ ] **Step 1: Author `proto/BUILD.bazel`**

```python
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@grpc//bazel:cc_grpc_library.bzl", "cc_grpc_library")
load("@protobuf//bazel:py_proto_library.bzl", "py_proto_library")
load("@grpc//bazel:python_rules.bzl", "py_grpc_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "radar_reprojector_proto",
    srcs = ["radar_reprojector.proto"],
)

cc_proto_library(
    name = "radar_reprojector_cc_proto",
    deps = [":radar_reprojector_proto"],
)

cc_grpc_library(
    name = "radar_reprojector_cc_grpc",
    srcs = [":radar_reprojector_proto"],
    deps = [":radar_reprojector_cc_proto"],
    grpc_only = True,
)

py_proto_library(
    name = "radar_reprojector_py_proto",
    deps = [":radar_reprojector_proto"],
)

py_grpc_library(
    name = "radar_reprojector_py_grpc",
    srcs = [":radar_reprojector_proto"],
    deps = [":radar_reprojector_py_proto"],
)
```

> **Note on rule paths.** `@protobuf//bazel:cc_proto_library.bzl` and `@grpc//bazel:cc_grpc_library.bzl` are the canonical bzlmod re-exports as of grpc 1.66 / protobuf 29. If a `load()` fails with "no such file", grep the resolved module's `bazel/` directory (`bazel query @grpc//bazel:*`) and adjust. Do not fall back to genrule + protoc by hand.

- [ ] **Step 2: Build the C++ codegen targets**

Run: `bazel build //proto:radar_reprojector_cc_grpc`
Expected: green build. Generated headers land in `bazel-bin/proto/radar_reprojector.{pb,grpc.pb}.h`.

- [ ] **Step 3: Build the Python codegen targets**

Run: `bazel build //proto:radar_reprojector_py_grpc`
Expected: green build. Generated `radar_reprojector_pb2.py` and `radar_reprojector_pb2_grpc.py` land under `bazel-bin/proto/`.

- [ ] **Step 4: Commit**

```bash
git add proto/BUILD.bazel
git commit -m "Wire proto codegen for C++ and Python"
```

---

## Task 3: Conversions header + `Vec3f` / `Pose` tests + impl

**Files:**
- Create: `server/Conversions.hpp`
- Create: `server/Conversions.cpp`
- Create: `server/test/Conversions_test.cpp`
- Create: `server/BUILD.bazel`

- [ ] **Step 1: Write the header**

`server/Conversions.hpp`:

```cpp
#ifndef RADAR_REPROJECTION_SERVER_CONVERSIONS_HPP_
#define RADAR_REPROJECTION_SERVER_CONVERSIONS_HPP_

#include <Eigen/Dense>

#include "algorithm/ActorTypes.hpp"
#include "algorithm/LogTypes.hpp"
#include "algorithm/RadarTypes.hpp"
#include "algorithm/ReprojectorTypes.hpp"

#include "proto/radar_reprojector.pb.h"

namespace radar_reprojection::server {

// Geometry primitives.
Eigen::Vector3f      fromProto(const proto::Vec3f& p);
proto::Vec3f         toProto(const Eigen::Vector3f& v);
Eigen::Affine3f      fromProto(const proto::Pose& p);
proto::Pose          toProto(const Eigen::Affine3f& a);

// Radar types.
algorithm::RadarIntrinsics fromProto(const proto::RadarIntrinsics& p);
proto::RadarIntrinsics     toProto(const algorithm::RadarIntrinsics& s);
algorithm::RadarParam      fromProto(const proto::RadarParam& p);
proto::RadarParam          toProto(const algorithm::RadarParam& s);
algorithm::ReferenceIndex  fromProto(const proto::ReferenceIndex& p);
proto::ReferenceIndex      toProto(const algorithm::ReferenceIndex& s);
algorithm::RadarDetection  fromProto(const proto::RadarDetection& p);
proto::RadarDetection      toProto(const algorithm::RadarDetection& s);
algorithm::RadarScan       fromProto(const proto::RadarScan& p);
proto::RadarScan           toProto(const algorithm::RadarScan& s);

// Actor types. NOTE: fromProto(ActorState) ignores the wire `from_world`
// field and recomputes it as `to_world.inverse()`.
algorithm::ActorState      fromProto(const proto::ActorState& p);
proto::ActorState          toProto(const algorithm::ActorState& s);

// Log types.
algorithm::LogRecording    fromProto(const proto::LogRecording& p);
proto::LogRecording        toProto(const algorithm::LogRecording& s);
algorithm::LogActorData    fromProto(const proto::LogActorData& p);
proto::LogActorData        toProto(const algorithm::LogActorData& s);
algorithm::LogData         fromProto(const proto::LogData& p);
proto::LogData             toProto(const algorithm::LogData& s);

// Top-level RPC payloads.
algorithm::ReprojectorParam fromProto(const proto::ReprojectorParam& p);
proto::ReprojectorParam     toProto(const algorithm::ReprojectorParam& s);
algorithm::ReprojectInput   fromProto(const proto::ReprojectRequest& p);

} // namespace radar_reprojection::server

#endif // RADAR_REPROJECTION_SERVER_CONVERSIONS_HPP_
```

- [ ] **Step 2: Write a stub `Conversions.cpp`**

`server/Conversions.cpp`:

```cpp
#include "server/Conversions.hpp"

namespace radar_reprojection::server {

Eigen::Vector3f fromProto(const proto::Vec3f& p) {
    return Eigen::Vector3f{p.x(), p.y(), p.z()};
}

proto::Vec3f toProto(const Eigen::Vector3f& v) {
    proto::Vec3f p;
    p.set_x(v.x());
    p.set_y(v.y());
    p.set_z(v.z());
    return p;
}

Eigen::Affine3f fromProto(const proto::Pose& p) {
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    t.translate(Eigen::Vector3f{p.translation().x(),
                                p.translation().y(),
                                p.translation().z()});
    t.rotate(Eigen::Quaternionf{p.rotation().w(),
                                p.rotation().x(),
                                p.rotation().y(),
                                p.rotation().z()});
    return t;
}

proto::Pose toProto(const Eigen::Affine3f& a) {
    Eigen::Quaternionf q{a.linear()};
    proto::Pose p;
    p.mutable_translation()->set_x(a.translation().x());
    p.mutable_translation()->set_y(a.translation().y());
    p.mutable_translation()->set_z(a.translation().z());
    p.mutable_rotation()->set_x(q.x());
    p.mutable_rotation()->set_y(q.y());
    p.mutable_rotation()->set_z(q.z());
    p.mutable_rotation()->set_w(q.w());
    return p;
}

} // namespace radar_reprojection::server
```

- [ ] **Step 3: Write the failing tests**

`server/test/Conversions_test.cpp`:

```cpp
#include "server/Conversions.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace rr  = radar_reprojection;
namespace rrp = radar_reprojection::proto;
namespace rra = radar_reprojection::algorithm;
namespace rrs = radar_reprojection::server;

namespace {

constexpr float kEps = 1e-5f;

void expectVec3fEq(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    EXPECT_NEAR(a.x(), b.x(), kEps);
    EXPECT_NEAR(a.y(), b.y(), kEps);
    EXPECT_NEAR(a.z(), b.z(), kEps);
}

void expectAffineEq(const Eigen::Affine3f& a, const Eigen::Affine3f& b) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            EXPECT_NEAR(a.matrix()(r, c), b.matrix()(r, c), kEps)
                << "(" << r << "," << c << ")";
        }
    }
}

} // namespace

TEST(Conversions, Vec3fRoundTrip) {
    Eigen::Vector3f orig{1.5f, -2.0f, 3.25f};
    expectVec3fEq(orig, rrs::fromProto(rrs::toProto(orig)));
}

TEST(Conversions, PoseIdentityRoundTrip) {
    Eigen::Affine3f orig = Eigen::Affine3f::Identity();
    expectAffineEq(orig, rrs::fromProto(rrs::toProto(orig)));
}

TEST(Conversions, PoseTranslationAndRotationRoundTrip) {
    Eigen::Affine3f orig = Eigen::Affine3f::Identity();
    orig.translate(Eigen::Vector3f{4.0f, -5.0f, 6.0f});
    orig.rotate(Eigen::AngleAxisf(0.7f, Eigen::Vector3f::UnitZ()));
    orig.rotate(Eigen::AngleAxisf(-0.3f, Eigen::Vector3f::UnitY()));
    expectAffineEq(orig, rrs::fromProto(rrs::toProto(orig)));
}
```

- [ ] **Step 4: Author `server/BUILD.bazel`**

```python
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

COMMON_COPTS = ["-Wall", "-Wextra", "-Wno-error=maybe-uninitialized", "-std=c++20"]

cc_library(
    name = "conversions",
    srcs = ["Conversions.cpp"],
    hdrs = ["Conversions.hpp"],
    copts = COMMON_COPTS,
    deps = [
        "//algorithm:algorithm",
        "//proto:radar_reprojector_cc_proto",
        "@eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "conversions_test",
    srcs = ["test/Conversions_test.cpp"],
    deps = [":conversions", "@googletest//:gtest_main"],
    copts = COMMON_COPTS,
)
```

- [ ] **Step 5: Run the test**

Run: `bazel test //server:conversions_test`
Expected: PASS — Vec3f round-trip + the two Pose round-trips.

- [ ] **Step 6: Commit**

```bash
git add server/Conversions.hpp server/Conversions.cpp \
        server/test/Conversions_test.cpp server/BUILD.bazel
git commit -m "Add server conversion layer: Vec3f and Pose round-trip"
```

---

## Task 4: ActorState conversions (with `from_world` recompute)

**Files:**
- Modify: `server/Conversions.cpp`
- Modify: `server/test/Conversions_test.cpp`

- [ ] **Step 1: Add the failing tests**

Append to `server/test/Conversions_test.cpp`:

```cpp
TEST(Conversions, ActorStateFromWorldIsRecomputed) {
    // Build a proto ActorState whose to_world is a real rotation+translation
    // but whose from_world is intentionally bogus. fromProto must IGNORE the
    // wire from_world and compute toWorld.inverse() instead.
    rrp::ActorState p;
    p.set_timestamp_us(1000);
    Eigen::Affine3f tw = Eigen::Affine3f::Identity();
    tw.translate(Eigen::Vector3f{2.0f, 0.0f, -1.0f});
    tw.rotate(Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ()));
    *p.mutable_to_world() = rrs::toProto(tw);
    // Bogus from_world: another arbitrary pose, not the inverse.
    Eigen::Affine3f bogus = Eigen::Affine3f::Identity();
    bogus.translate(Eigen::Vector3f{99.0f, 99.0f, 99.0f});
    *p.mutable_from_world() = rrs::toProto(bogus);
    *p.mutable_velocity_mps()     = rrs::toProto(Eigen::Vector3f{1.0f, 0.0f, 0.0f});
    *p.mutable_bbox_half_size_m() = rrs::toProto(Eigen::Vector3f{2.0f, 1.0f, 0.75f});
    *p.mutable_bbox_center_m()    = rrs::toProto(Eigen::Vector3f::Zero());

    rra::ActorState a = rrs::fromProto(p);
    Eigen::Affine3f expectedFromWorld = tw.inverse();
    expectAffineEq(a.fromWorld, expectedFromWorld);
    // toWorld matches what we sent.
    expectAffineEq(a.toWorld, tw);
    EXPECT_EQ(a.timestampUs, 1000);
}

TEST(Conversions, ActorStateRoundTripWithConsistentFromWorld) {
    rra::ActorState orig{};
    orig.timestampUs = 5000;
    orig.toWorld     = Eigen::Affine3f::Identity();
    orig.toWorld.translate(Eigen::Vector3f{1.0f, 2.0f, 3.0f});
    orig.toWorld.rotate(Eigen::AngleAxisf(0.2f, Eigen::Vector3f::UnitX()));
    orig.fromWorld   = orig.toWorld.inverse();
    orig.velocityMps   = Eigen::Vector3f{0.1f, 0.2f, 0.3f};
    orig.bboxHalfSizeM = Eigen::Vector3f{2.0f, 1.0f, 0.5f};
    orig.bboxCenterM   = Eigen::Vector3f{0.0f, 0.0f, 0.0f};

    rra::ActorState rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_EQ(rt.timestampUs, orig.timestampUs);
    expectAffineEq(rt.toWorld,    orig.toWorld);
    expectAffineEq(rt.fromWorld,  orig.fromWorld);
    expectVec3fEq(rt.velocityMps,   orig.velocityMps);
    expectVec3fEq(rt.bboxHalfSizeM, orig.bboxHalfSizeM);
    expectVec3fEq(rt.bboxCenterM,   orig.bboxCenterM);
}
```

- [ ] **Step 2: Run tests, expect failure**

Run: `bazel test //server:conversions_test`
Expected: FAIL — `fromProto(ActorState)` and `toProto(ActorState)` don't exist yet, so it should fail at link time.

- [ ] **Step 3: Add the implementations**

Append to `server/Conversions.cpp` (before the closing namespace):

```cpp
algorithm::ActorState fromProto(const proto::ActorState& p) {
    algorithm::ActorState s{};
    s.timestampUs = p.timestamp_us();
    s.toWorld     = fromProto(p.to_world());
    // Spec §5: from_world is recomputed, never trusted.
    s.fromWorld   = s.toWorld.inverse();
    s.velocityMps   = fromProto(p.velocity_mps());
    s.bboxHalfSizeM = fromProto(p.bbox_half_size_m());
    s.bboxCenterM   = fromProto(p.bbox_center_m());
    return s;
}

proto::ActorState toProto(const algorithm::ActorState& s) {
    proto::ActorState p;
    p.set_timestamp_us(s.timestampUs);
    *p.mutable_to_world()         = toProto(s.toWorld);
    *p.mutable_from_world()       = toProto(s.fromWorld);
    *p.mutable_velocity_mps()     = toProto(s.velocityMps);
    *p.mutable_bbox_half_size_m() = toProto(s.bboxHalfSizeM);
    *p.mutable_bbox_center_m()    = toProto(s.bboxCenterM);
    return p;
}
```

- [ ] **Step 4: Run tests, expect pass**

Run: `bazel test //server:conversions_test`
Expected: PASS — all five tests now green.

- [ ] **Step 5: Commit**

```bash
git add server/Conversions.cpp server/test/Conversions_test.cpp
git commit -m "ActorState conversions; from_world recomputed server-side"
```

---

## Task 5: Radar-types conversions (Intrinsics, Param, ReferenceIndex, Detection, Scan)

**Files:**
- Modify: `server/Conversions.cpp`
- Modify: `server/test/Conversions_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `server/test/Conversions_test.cpp`:

```cpp
TEST(Conversions, RadarIntrinsicsRoundTrip) {
    rra::RadarIntrinsics orig{};
    orig.minRangeM   = 0.2f;
    orig.maxRangeM   = 200.0f;
    orig.halfHfovRad = 1.2f;
    orig.halfVfovRad = 0.4f;
    orig.hfovResRad  = 0.01f;
    orig.vfovResRad  = 0.02f;
    orig.rangeResM   = 0.15f;
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_FLOAT_EQ(rt.minRangeM,   orig.minRangeM);
    EXPECT_FLOAT_EQ(rt.maxRangeM,   orig.maxRangeM);
    EXPECT_FLOAT_EQ(rt.halfHfovRad, orig.halfHfovRad);
    EXPECT_FLOAT_EQ(rt.halfVfovRad, orig.halfVfovRad);
    EXPECT_FLOAT_EQ(rt.hfovResRad,  orig.hfovResRad);
    EXPECT_FLOAT_EQ(rt.vfovResRad,  orig.vfovResRad);
    EXPECT_FLOAT_EQ(rt.rangeResM,   orig.rangeResM);
}

TEST(Conversions, RadarParamRoundTrip) {
    rra::RadarParam orig{};
    orig.id = "radar:front:center";
    orig.intrinsics.minRangeM = 0.5f;
    orig.toEgo = Eigen::Affine3f::Identity();
    orig.toEgo.translate(Eigen::Vector3f{1.0f, 0.0f, 0.5f});
    orig.maxNumReturns = 256;
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_EQ(rt.id, orig.id);
    EXPECT_FLOAT_EQ(rt.intrinsics.minRangeM, orig.intrinsics.minRangeM);
    expectAffineEq(rt.toEgo, orig.toEgo);
    EXPECT_EQ(rt.maxNumReturns, orig.maxNumReturns);
}

TEST(Conversions, ReferenceIndexRoundTrip) {
    rra::ReferenceIndex orig{"radar:front:center", 12, 34, "actor:42"};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_EQ(rt.radarId,        orig.radarId);
    EXPECT_EQ(rt.scanIndex,      orig.scanIndex);
    EXPECT_EQ(rt.detectionIndex, orig.detectionIndex);
    EXPECT_EQ(rt.actorId,        orig.actorId);
}

TEST(Conversions, RadarDetectionRoundTrip) {
    rra::RadarDetection orig{};
    orig.azimuthRad        = 0.1f;
    orig.elevationRad      = -0.05f;
    orig.distanceM         = 42.0f;
    orig.radialVelocityMps = -3.5f;
    orig.rcsDbm            = 12.0f;
    orig.reference         = {"radar:front:center", 0, 7, ""};
    orig.velInWorld        = Eigen::Vector3f{0.5f, 0.0f, 0.0f};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_FLOAT_EQ(rt.azimuthRad,        orig.azimuthRad);
    EXPECT_FLOAT_EQ(rt.elevationRad,      orig.elevationRad);
    EXPECT_FLOAT_EQ(rt.distanceM,         orig.distanceM);
    EXPECT_FLOAT_EQ(rt.radialVelocityMps, orig.radialVelocityMps);
    EXPECT_FLOAT_EQ(rt.rcsDbm,            orig.rcsDbm);
    EXPECT_EQ(rt.reference.scanIndex,     orig.reference.scanIndex);
    expectVec3fEq(rt.velInWorld, orig.velInWorld);
}

TEST(Conversions, RadarScanRoundTripWithDetections) {
    rra::RadarScan orig{};
    orig.timestampUs = 1234567;
    rra::RadarDetection d{};
    d.azimuthRad = 0.0f; d.distanceM = 1.0f; d.rcsDbm = 0.0f;
    orig.detections = {d, d};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_EQ(rt.timestampUs, orig.timestampUs);
    ASSERT_EQ(rt.detections.size(), 2u);
    EXPECT_FLOAT_EQ(rt.detections[0].distanceM, 1.0f);
}

TEST(Conversions, RadarScanEmptyDetections) {
    rra::RadarScan orig{};
    orig.timestampUs = 0;
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_TRUE(rt.detections.empty());
}
```

- [ ] **Step 2: Run, expect failures**

Run: `bazel test //server:conversions_test`
Expected: FAIL (link errors for missing functions).

- [ ] **Step 3: Implement them in `Conversions.cpp`**

Append before the closing namespace:

```cpp
algorithm::RadarIntrinsics fromProto(const proto::RadarIntrinsics& p) {
    algorithm::RadarIntrinsics s{};
    s.minRangeM   = p.min_range_m();
    s.maxRangeM   = p.max_range_m();
    s.halfHfovRad = p.half_hfov_rad();
    s.halfVfovRad = p.half_vfov_rad();
    s.hfovResRad  = p.hfov_res_rad();
    s.vfovResRad  = p.vfov_res_rad();
    s.rangeResM   = p.range_res_m();
    return s;
}

proto::RadarIntrinsics toProto(const algorithm::RadarIntrinsics& s) {
    proto::RadarIntrinsics p;
    p.set_min_range_m(s.minRangeM);
    p.set_max_range_m(s.maxRangeM);
    p.set_half_hfov_rad(s.halfHfovRad);
    p.set_half_vfov_rad(s.halfVfovRad);
    p.set_hfov_res_rad(s.hfovResRad);
    p.set_vfov_res_rad(s.vfovResRad);
    p.set_range_res_m(s.rangeResM);
    return p;
}

algorithm::RadarParam fromProto(const proto::RadarParam& p) {
    algorithm::RadarParam s{};
    s.id            = p.id();
    s.intrinsics    = fromProto(p.intrinsics());
    s.toEgo         = fromProto(p.to_ego());
    s.maxNumReturns = p.max_num_returns();
    return s;
}

proto::RadarParam toProto(const algorithm::RadarParam& s) {
    proto::RadarParam p;
    p.set_id(s.id);
    *p.mutable_intrinsics() = toProto(s.intrinsics);
    *p.mutable_to_ego()     = toProto(s.toEgo);
    p.set_max_num_returns(s.maxNumReturns);
    return p;
}

algorithm::ReferenceIndex fromProto(const proto::ReferenceIndex& p) {
    return algorithm::ReferenceIndex{
        p.radar_id(), p.scan_index(), p.detection_index(), p.actor_id()};
}

proto::ReferenceIndex toProto(const algorithm::ReferenceIndex& s) {
    proto::ReferenceIndex p;
    p.set_radar_id(s.radarId);
    p.set_scan_index(s.scanIndex);
    p.set_detection_index(s.detectionIndex);
    p.set_actor_id(s.actorId);
    return p;
}

algorithm::RadarDetection fromProto(const proto::RadarDetection& p) {
    algorithm::RadarDetection s{};
    s.azimuthRad        = p.azimuth_rad();
    s.elevationRad      = p.elevation_rad();
    s.distanceM         = p.distance_m();
    s.radialVelocityMps = p.radial_velocity_mps();
    s.rcsDbm            = p.rcs_dbm();
    s.reference         = fromProto(p.reference());
    s.velInWorld        = fromProto(p.vel_in_world());
    return s;
}

proto::RadarDetection toProto(const algorithm::RadarDetection& s) {
    proto::RadarDetection p;
    p.set_azimuth_rad(s.azimuthRad);
    p.set_elevation_rad(s.elevationRad);
    p.set_distance_m(s.distanceM);
    p.set_radial_velocity_mps(s.radialVelocityMps);
    p.set_rcs_dbm(s.rcsDbm);
    *p.mutable_reference()    = toProto(s.reference);
    *p.mutable_vel_in_world() = toProto(s.velInWorld);
    return p;
}

algorithm::RadarScan fromProto(const proto::RadarScan& p) {
    algorithm::RadarScan s{};
    s.timestampUs = p.timestamp_us();
    s.detections.reserve(p.detections_size());
    for (const auto& d : p.detections()) {
        s.detections.push_back(fromProto(d));
    }
    return s;
}

proto::RadarScan toProto(const algorithm::RadarScan& s) {
    proto::RadarScan p;
    p.set_timestamp_us(s.timestampUs);
    for (const auto& d : s.detections) {
        *p.add_detections() = toProto(d);
    }
    return p;
}
```

- [ ] **Step 4: Run, expect pass**

Run: `bazel test //server:conversions_test`
Expected: PASS — all radar-type tests green.

- [ ] **Step 5: Commit**

```bash
git add server/Conversions.cpp server/test/Conversions_test.cpp
git commit -m "Radar-types conversions and round-trip tests"
```

---

## Task 6: LogData conversions (LogRecording, LogActorData, LogData)

**Files:**
- Modify: `server/Conversions.cpp`
- Modify: `server/test/Conversions_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `server/test/Conversions_test.cpp`:

```cpp
namespace {

rra::ActorState consistentActor(int64_t ts, float tx) {
    rra::ActorState s{};
    s.timestampUs = ts;
    s.toWorld     = Eigen::Affine3f::Identity();
    s.toWorld.translate(Eigen::Vector3f{tx, 0.0f, 0.0f});
    s.fromWorld   = s.toWorld.inverse();
    s.velocityMps   = Eigen::Vector3f::Zero();
    s.bboxHalfSizeM = Eigen::Vector3f{2.0f, 1.0f, 0.5f};
    s.bboxCenterM   = Eigen::Vector3f::Zero();
    return s;
}

} // namespace

TEST(Conversions, LogRecordingRoundTrip) {
    rra::LogRecording orig{};
    orig.param.id = "radar:front:center";
    orig.param.intrinsics.minRangeM = 0.5f;
    rra::RadarScan scan{};
    scan.timestampUs = 1000;
    orig.scans = {scan};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_EQ(rt.param.id, orig.param.id);
    ASSERT_EQ(rt.scans.size(), 1u);
    EXPECT_EQ(rt.scans[0].timestampUs, 1000);
}

TEST(Conversions, LogActorDataRoundTrip) {
    rra::LogActorData orig{};
    orig.egoTrajectory = {consistentActor(0, 0.0f), consistentActor(1000, 0.5f)};
    orig.actorToTrajectory["actor:42"] = {consistentActor(0, 10.0f)};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    ASSERT_EQ(rt.egoTrajectory.size(), 2u);
    EXPECT_EQ(rt.egoTrajectory[1].timestampUs, 1000);
    ASSERT_EQ(rt.actorToTrajectory.count("actor:42"), 1u);
    EXPECT_EQ(rt.actorToTrajectory.at("actor:42").size(), 1u);
}

TEST(Conversions, LogDataEmptyRoundTrip) {
    rra::LogData orig{};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_TRUE(rt.logRadarData.empty());
    EXPECT_TRUE(rt.logActorData.egoTrajectory.empty());
    EXPECT_TRUE(rt.logActorData.actorToTrajectory.empty());
}

TEST(Conversions, LogDataPopulatedRoundTrip) {
    rra::LogData orig{};
    rra::LogRecording rec{};
    rec.param.id = "r0";
    orig.logRadarData["r0"] = rec;
    orig.logActorData.egoTrajectory = {consistentActor(0, 0.0f)};
    auto rt = rrs::fromProto(rrs::toProto(orig));
    ASSERT_EQ(rt.logRadarData.count("r0"), 1u);
    EXPECT_EQ(rt.logRadarData.at("r0").param.id, "r0");
    EXPECT_EQ(rt.logActorData.egoTrajectory.size(), 1u);
}
```

- [ ] **Step 2: Run, expect failures**

Run: `bazel test //server:conversions_test`
Expected: FAIL — link errors.

- [ ] **Step 3: Implement them**

Append to `server/Conversions.cpp`:

```cpp
algorithm::LogRecording fromProto(const proto::LogRecording& p) {
    algorithm::LogRecording s{};
    s.param = fromProto(p.param());
    s.scans.reserve(p.scans_size());
    for (const auto& sc : p.scans()) {
        s.scans.push_back(fromProto(sc));
    }
    return s;
}

proto::LogRecording toProto(const algorithm::LogRecording& s) {
    proto::LogRecording p;
    *p.mutable_param() = toProto(s.param);
    for (const auto& sc : s.scans) {
        *p.add_scans() = toProto(sc);
    }
    return p;
}

algorithm::LogActorData fromProto(const proto::LogActorData& p) {
    algorithm::LogActorData s{};
    s.egoTrajectory.reserve(p.ego_trajectory().states_size());
    for (const auto& st : p.ego_trajectory().states()) {
        s.egoTrajectory.push_back(fromProto(st));
    }
    for (const auto& [actorId, traj] : p.actor_to_trajectory()) {
        algorithm::ActorTrajectory out;
        out.reserve(traj.states_size());
        for (const auto& st : traj.states()) {
            out.push_back(fromProto(st));
        }
        s.actorToTrajectory.emplace(actorId, std::move(out));
    }
    return s;
}

proto::LogActorData toProto(const algorithm::LogActorData& s) {
    proto::LogActorData p;
    auto* ego = p.mutable_ego_trajectory();
    for (const auto& st : s.egoTrajectory) {
        *ego->add_states() = toProto(st);
    }
    auto& outMap = *p.mutable_actor_to_trajectory();
    for (const auto& [actorId, traj] : s.actorToTrajectory) {
        proto::ActorTrajectory pt;
        for (const auto& st : traj) {
            *pt.add_states() = toProto(st);
        }
        outMap[actorId] = std::move(pt);
    }
    return p;
}

algorithm::LogData fromProto(const proto::LogData& p) {
    algorithm::LogData s{};
    for (const auto& [radarId, rec] : p.log_radar_data()) {
        s.logRadarData.emplace(radarId, fromProto(rec));
    }
    s.logActorData = fromProto(p.log_actor_data());
    return s;
}

proto::LogData toProto(const algorithm::LogData& s) {
    proto::LogData p;
    auto& outMap = *p.mutable_log_radar_data();
    for (const auto& [radarId, rec] : s.logRadarData) {
        outMap[radarId] = toProto(rec);
    }
    *p.mutable_log_actor_data() = toProto(s.logActorData);
    return p;
}
```

- [ ] **Step 4: Run, expect pass**

Run: `bazel test //server:conversions_test`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add server/Conversions.cpp server/test/Conversions_test.cpp
git commit -m "LogData / LogRecording / LogActorData conversions"
```

---

## Task 7: ReprojectorParam + ReprojectRequest → ReprojectInput

**Files:**
- Modify: `server/Conversions.cpp`
- Modify: `server/test/Conversions_test.cpp`

- [ ] **Step 1: Write the failing tests**

Append:

```cpp
TEST(Conversions, ReprojectorParamRoundTrip) {
    rra::ReprojectorParam orig{};
    orig.useLogIntrinsics = true;
    orig.doDownSampling   = false;
    orig.bboxScaleFactor  = Eigen::Vector3f{1.25f, 1.25f, 2.0f};
    orig.bboxMargin       = Eigen::Vector3f{0.25f, 0.25f, 1.0f};
    orig.maxSearchSize    = 30;
    orig.dump             = false;
    orig.dumpFolder       = "/tmp/x";
    auto rt = rrs::fromProto(rrs::toProto(orig));
    EXPECT_EQ(rt.useLogIntrinsics, orig.useLogIntrinsics);
    EXPECT_EQ(rt.doDownSampling,   orig.doDownSampling);
    expectVec3fEq(rt.bboxScaleFactor, orig.bboxScaleFactor);
    expectVec3fEq(rt.bboxMargin,      orig.bboxMargin);
    EXPECT_EQ(rt.maxSearchSize, orig.maxSearchSize);
    EXPECT_EQ(rt.dump,          orig.dump);
    EXPECT_EQ(rt.dumpFolder,    orig.dumpFolder);
}

TEST(Conversions, ReprojectRequestToInputFlattens) {
    rrp::ReprojectRequest req;
    req.set_timestamp_us(99);
    auto* rp = req.mutable_radar_param();
    rp->set_id("r0");
    rp->set_max_num_returns(64);
    *rp->mutable_to_ego() = rrs::toProto(Eigen::Affine3f::Identity());
    rra::ActorState ego = consistentActor(99, 0.0f);
    *req.mutable_ego_state() = rrs::toProto(ego);
    auto& actors = *req.mutable_actor_to_state();
    actors["a:0"] = rrs::toProto(consistentActor(99, 5.0f));

    auto in = rrs::fromProto(req);
    EXPECT_EQ(in.timestampUs, 99);
    EXPECT_EQ(in.radarParam.id, "r0");
    EXPECT_EQ(in.radarParam.maxNumReturns, 64u);
    EXPECT_EQ(in.egoState.timestampUs, 99);
    ASSERT_EQ(in.actorToState.count("a:0"), 1u);
    EXPECT_EQ(in.actorToState.at("a:0").timestampUs, 99);
}
```

- [ ] **Step 2: Run, expect failures**

Run: `bazel test //server:conversions_test`
Expected: FAIL — missing functions.

- [ ] **Step 3: Implement them**

Append to `server/Conversions.cpp`:

```cpp
algorithm::ReprojectorParam fromProto(const proto::ReprojectorParam& p) {
    algorithm::ReprojectorParam s{};
    s.useLogIntrinsics = p.use_log_intrinsics();
    s.doDownSampling   = p.do_down_sampling();
    s.bboxScaleFactor  = fromProto(p.bbox_scale_factor());
    s.bboxMargin       = fromProto(p.bbox_margin());
    s.maxSearchSize    = static_cast<size_t>(p.max_search_size());
    s.dump             = p.dump();
    s.dumpFolder       = p.dump_folder();
    return s;
}

proto::ReprojectorParam toProto(const algorithm::ReprojectorParam& s) {
    proto::ReprojectorParam p;
    p.set_use_log_intrinsics(s.useLogIntrinsics);
    p.set_do_down_sampling(s.doDownSampling);
    *p.mutable_bbox_scale_factor() = toProto(s.bboxScaleFactor);
    *p.mutable_bbox_margin()       = toProto(s.bboxMargin);
    p.set_max_search_size(static_cast<uint64_t>(s.maxSearchSize));
    p.set_dump(s.dump);
    p.set_dump_folder(s.dumpFolder);
    return p;
}

algorithm::ReprojectInput fromProto(const proto::ReprojectRequest& p) {
    algorithm::ReprojectInput in{};
    in.timestampUs = p.timestamp_us();
    in.radarParam  = fromProto(p.radar_param());
    in.egoState    = fromProto(p.ego_state());
    for (const auto& [actorId, st] : p.actor_to_state()) {
        in.actorToState.emplace(actorId, fromProto(st));
    }
    return in;
}
```

- [ ] **Step 4: Run, expect pass**

Run: `bazel test //server:conversions_test`
Expected: PASS — all conversion tests green.

- [ ] **Step 5: Commit**

```bash
git add server/Conversions.cpp server/test/Conversions_test.cpp
git commit -m "ReprojectorParam + ReprojectRequest -> ReprojectInput conversion"
```

---

## Task 8: Server class skeleton + `Reproject` before `Initialize` test

**Files:**
- Create: `server/RadarReprojectionServer.hpp`
- Create: `server/RadarReprojectionServer.cpp`
- Create: `server/test/RadarReprojectionServer_test.cpp`
- Modify: `server/BUILD.bazel`

- [ ] **Step 1: Write the header**

`server/RadarReprojectionServer.hpp`:

```cpp
#ifndef RADAR_REPROJECTION_SERVER_RADAR_REPROJECTION_SERVER_HPP_
#define RADAR_REPROJECTION_SERVER_RADAR_REPROJECTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <unordered_map>

#include <grpcpp/grpcpp.h>

#include "algorithm/LogTypes.hpp"
#include "algorithm/RadarReprojector.hpp"
#include "algorithm/RadarTypes.hpp"

#include "proto/radar_reprojector.grpc.pb.h"

namespace radar_reprojection::server {

class RadarReprojectionServer final
    : public proto::RadarReprojectorService::Service {
public:
    RadarReprojectionServer();

    grpc::Status Initialize(grpc::ServerContext* ctx,
                            const proto::InitializeRequest* req,
                            proto::InitializeResponse* resp) override;

    grpc::Status Reproject(grpc::ServerContext* ctx,
                           const proto::ReprojectRequest* req,
                           proto::ReprojectResponse* resp) override;

private:
    std::mutex                                                m_mutex;
    std::unique_ptr<algorithm::RadarReprojector>              m_reprojector;
    std::shared_ptr<algorithm::LogData>                       m_logData;
    std::unordered_map<std::string, algorithm::RadarParam>    m_radarParams;
};

} // namespace radar_reprojection::server

#endif // RADAR_REPROJECTION_SERVER_RADAR_REPROJECTION_SERVER_HPP_
```

- [ ] **Step 2: Write a stub `RadarReprojectionServer.cpp`**

```cpp
#include "server/RadarReprojectionServer.hpp"

namespace radar_reprojection::server {

RadarReprojectionServer::RadarReprojectionServer() = default;

grpc::Status RadarReprojectionServer::Initialize(
    grpc::ServerContext* /*ctx*/,
    const proto::InitializeRequest* /*req*/,
    proto::InitializeResponse* /*resp*/) {
    // Implemented in Task 9.
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED, "Initialize");
}

grpc::Status RadarReprojectionServer::Reproject(
    grpc::ServerContext* /*ctx*/,
    const proto::ReprojectRequest* /*req*/,
    proto::ReprojectResponse* /*resp*/) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_reprojector) {
        return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                            "Reproject called before Initialize");
    }
    return grpc::Status(grpc::StatusCode::UNIMPLEMENTED, "Reproject");
}

} // namespace radar_reprojection::server
```

- [ ] **Step 3: Write the failing test**

`server/test/RadarReprojectionServer_test.cpp`:

```cpp
#include "server/RadarReprojectionServer.hpp"

#include <gtest/gtest.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

namespace rrp = radar_reprojection::proto;
namespace rrs = radar_reprojection::server;

namespace {

struct ServerFixture : public ::testing::Test {
    rrs::RadarReprojectionServer service;
    std::unique_ptr<grpc::Server> server;
    std::shared_ptr<grpc::Channel> channel;
    std::unique_ptr<rrp::RadarReprojectorService::Stub> stub;

    void SetUp() override {
        grpc::ServerBuilder builder;
        builder.RegisterService(&service);
        server  = builder.BuildAndStart();
        channel = server->InProcessChannel(grpc::ChannelArguments{});
        stub    = rrp::RadarReprojectorService::NewStub(channel);
    }

    void TearDown() override {
        server->Shutdown();
        server->Wait();
    }
};

} // namespace

TEST_F(ServerFixture, ReprojectBeforeInitializeFailsPrecondition) {
    rrp::ReprojectRequest req;
    rrp::ReprojectResponse resp;
    grpc::ClientContext ctx;
    grpc::Status s = stub->Reproject(&ctx, req, &resp);
    EXPECT_EQ(s.error_code(), grpc::StatusCode::FAILED_PRECONDITION);
}
```

- [ ] **Step 4: Update `server/BUILD.bazel`**

Append two new entries:

```python
cc_library(
    name = "radar_reprojection_server_lib",
    srcs = ["RadarReprojectionServer.cpp"],
    hdrs = ["RadarReprojectionServer.hpp"],
    copts = COMMON_COPTS,
    deps = [
        ":conversions",
        "//algorithm:algorithm",
        "//proto:radar_reprojector_cc_grpc",
        "@grpc//:grpc++",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "radar_reprojection_server_test",
    srcs = ["test/RadarReprojectionServer_test.cpp"],
    deps = [
        ":radar_reprojection_server_lib",
        "@grpc//:grpc++",
        "@googletest//:gtest_main",
    ],
    copts = COMMON_COPTS,
)
```

- [ ] **Step 5: Run the new test**

Run: `bazel test //server:radar_reprojection_server_test`
Expected: PASS — `Reproject` returns `FAILED_PRECONDITION` because the reprojector is null.

- [ ] **Step 6: Commit**

```bash
git add server/RadarReprojectionServer.hpp server/RadarReprojectionServer.cpp \
        server/test/RadarReprojectionServer_test.cpp server/BUILD.bazel
git commit -m "Server skeleton; FAILED_PRECONDITION when Reproject precedes Initialize"
```

---

## Task 9: Implement `Initialize` and a happy-path test

**Files:**
- Modify: `server/RadarReprojectionServer.cpp`
- Modify: `server/test/RadarReprojectionServer_test.cpp`

- [ ] **Step 1: Replace the `Initialize` body**

In `server/RadarReprojectionServer.cpp`, replace the stub. Add `#include "server/Conversions.hpp"` near the top, then:

```cpp
grpc::Status RadarReprojectionServer::Initialize(
    grpc::ServerContext* /*ctx*/,
    const proto::InitializeRequest* req,
    proto::InitializeResponse* /*resp*/) {
    std::lock_guard<std::mutex> lock(m_mutex);
    try {
        algorithm::ReprojectorParam param = fromProto(req->param());
        auto logData = std::make_shared<algorithm::LogData>(
            fromProto(req->log_data()));

        auto reprojector = std::make_unique<algorithm::RadarReprojector>();
        if (!reprojector->initialize(param, logData)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "RadarReprojector::initialize returned false");
        }

        std::unordered_map<std::string, algorithm::RadarParam> radarParams;
        for (const auto& [radarId, recording] : logData->logRadarData) {
            radarParams.emplace(radarId, recording.param);
        }

        m_reprojector  = std::move(reprojector);
        m_logData      = std::move(logData);
        m_radarParams  = std::move(radarParams);
        return grpc::Status::OK;
    } catch (const std::exception& e) {
        return grpc::Status(grpc::StatusCode::INTERNAL, e.what());
    }
}
```

- [ ] **Step 2: Write the new tests**

Append helper + tests to `server/test/RadarReprojectionServer_test.cpp`:

```cpp
namespace {

rrp::InitializeRequest minimalInitializeRequest() {
    rrp::InitializeRequest req;
    auto* param = req.mutable_param();
    param->set_use_log_intrinsics(true);
    param->set_do_down_sampling(false);
    param->set_max_search_size(1);
    param->set_dump(false);
    auto* bsf = param->mutable_bbox_scale_factor();
    bsf->set_x(1.0f); bsf->set_y(1.0f); bsf->set_z(1.0f);
    auto* bm = param->mutable_bbox_margin();
    bm->set_x(0.0f); bm->set_y(0.0f); bm->set_z(0.0f);

    auto* logData = req.mutable_log_data();
    auto* radars  = logData->mutable_log_radar_data();
    rrp::LogRecording rec;
    rec.mutable_param()->set_id("r0");
    rec.mutable_param()->set_max_num_returns(8);
    auto* intr = rec.mutable_param()->mutable_intrinsics();
    intr->set_min_range_m(0.1f);
    intr->set_max_range_m(100.0f);
    intr->set_half_hfov_rad(1.0f);
    intr->set_half_vfov_rad(0.5f);
    intr->set_hfov_res_rad(0.01f);
    intr->set_vfov_res_rad(0.01f);
    intr->set_range_res_m(0.1f);
    auto* toEgo = rec.mutable_param()->mutable_to_ego();
    toEgo->mutable_translation()->set_x(0.0f);
    toEgo->mutable_translation()->set_y(0.0f);
    toEgo->mutable_translation()->set_z(0.0f);
    toEgo->mutable_rotation()->set_w(1.0f);
    toEgo->mutable_rotation()->set_x(0.0f);
    toEgo->mutable_rotation()->set_y(0.0f);
    toEgo->mutable_rotation()->set_z(0.0f);
    auto* scan = rec.add_scans();
    scan->set_timestamp_us(0);
    (*radars)["r0"] = rec;

    auto* ego = logData->mutable_log_actor_data()->mutable_ego_trajectory();
    auto* st  = ego->add_states();
    st->set_timestamp_us(0);
    auto* tw  = st->mutable_to_world();
    tw->mutable_rotation()->set_w(1.0f);
    auto* fw  = st->mutable_from_world();
    fw->mutable_rotation()->set_w(1.0f);
    auto* vmps = st->mutable_velocity_mps(); (void)vmps;
    auto* hbox = st->mutable_bbox_half_size_m();
    hbox->set_x(2.0f); hbox->set_y(1.0f); hbox->set_z(0.5f);
    auto* cbox = st->mutable_bbox_center_m(); (void)cbox;
    return req;
}

} // namespace

TEST_F(ServerFixture, InitializeSucceedsWithMinimalLogData) {
    rrp::InitializeRequest req = minimalInitializeRequest();
    rrp::InitializeResponse resp;
    grpc::ClientContext ctx;
    grpc::Status s = stub->Initialize(&ctx, req, &resp);
    EXPECT_TRUE(s.ok()) << s.error_code() << ": " << s.error_message();
}
```

- [ ] **Step 3: Run the tests**

Run: `bazel test //server:radar_reprojection_server_test`
Expected: PASS — both tests green. (The `RadarReprojector::initialize()` call succeeds for trivial input; if it fails for some algorithm-internal reason, the `EXPECT_TRUE(s.ok())` will report the message — adjust the fixture by adding more scans/actors as needed; do **not** mark the test "expected to fail" without diagnosing first.)

- [ ] **Step 4: Commit**

```bash
git add server/RadarReprojectionServer.cpp server/test/RadarReprojectionServer_test.cpp
git commit -m "Implement Initialize RPC; happy-path in-process test"
```

---

## Task 10: Implement `Reproject` and the radar-id-validation test

**Files:**
- Modify: `server/RadarReprojectionServer.cpp`
- Modify: `server/test/RadarReprojectionServer_test.cpp`

- [ ] **Step 1: Replace the `Reproject` body**

In `server/RadarReprojectionServer.cpp`, replace the stub:

```cpp
grpc::Status RadarReprojectionServer::Reproject(
    grpc::ServerContext* /*ctx*/,
    const proto::ReprojectRequest* req,
    proto::ReprojectResponse* resp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_reprojector) {
        return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                            "Reproject called before Initialize");
    }
    if (m_radarParams.find(req->radar_param().id()) == m_radarParams.end()) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
            "unknown radarId: " + req->radar_param().id());
    }
    try {
        algorithm::ReprojectInput input = fromProto(*req);
        algorithm::RadarScan scan = m_reprojector->reproject(input);
        *resp->mutable_scan() = toProto(scan);
        return grpc::Status::OK;
    } catch (const std::exception& e) {
        return grpc::Status(grpc::StatusCode::INTERNAL, e.what());
    }
}
```

- [ ] **Step 2: Add tests**

Append:

```cpp
TEST_F(ServerFixture, ReprojectAfterInitializeReturnsScan) {
    rrp::InitializeResponse iresp;
    {
        grpc::ClientContext ctx;
        ASSERT_TRUE(stub->Initialize(&ctx, minimalInitializeRequest(), &iresp).ok());
    }

    rrp::ReprojectRequest req;
    req.set_timestamp_us(0);
    auto* rp = req.mutable_radar_param();
    rp->set_id("r0");
    rp->set_max_num_returns(8);
    rp->mutable_to_ego()->mutable_rotation()->set_w(1.0f);
    auto* intr = rp->mutable_intrinsics();
    intr->set_min_range_m(0.1f);
    intr->set_max_range_m(100.0f);
    intr->set_half_hfov_rad(1.0f);
    intr->set_half_vfov_rad(0.5f);
    intr->set_hfov_res_rad(0.01f);
    intr->set_vfov_res_rad(0.01f);
    intr->set_range_res_m(0.1f);
    auto* ego = req.mutable_ego_state();
    ego->set_timestamp_us(0);
    ego->mutable_to_world()->mutable_rotation()->set_w(1.0f);
    ego->mutable_from_world()->mutable_rotation()->set_w(1.0f);
    ego->mutable_bbox_half_size_m()->set_x(2.0f);
    ego->mutable_bbox_half_size_m()->set_y(1.0f);
    ego->mutable_bbox_half_size_m()->set_z(0.5f);

    rrp::ReprojectResponse resp;
    grpc::ClientContext ctx;
    grpc::Status s = stub->Reproject(&ctx, req, &resp);
    EXPECT_TRUE(s.ok()) << s.error_code() << ": " << s.error_message();
}

TEST_F(ServerFixture, ReprojectUnknownRadarIdInvalidArgument) {
    rrp::InitializeResponse iresp;
    {
        grpc::ClientContext ctx;
        ASSERT_TRUE(stub->Initialize(&ctx, minimalInitializeRequest(), &iresp).ok());
    }

    rrp::ReprojectRequest req;
    req.set_timestamp_us(0);
    req.mutable_radar_param()->set_id("does-not-exist");
    req.mutable_radar_param()->mutable_to_ego()->mutable_rotation()->set_w(1.0f);
    req.mutable_ego_state()->mutable_to_world()->mutable_rotation()->set_w(1.0f);

    rrp::ReprojectResponse resp;
    grpc::ClientContext ctx;
    grpc::Status s = stub->Reproject(&ctx, req, &resp);
    EXPECT_EQ(s.error_code(), grpc::StatusCode::INVALID_ARGUMENT);
}

TEST_F(ServerFixture, SecondInitializeReplacesState) {
    rrp::InitializeResponse iresp;
    {
        grpc::ClientContext ctx;
        ASSERT_TRUE(stub->Initialize(&ctx, minimalInitializeRequest(), &iresp).ok());
    }
    {
        grpc::ClientContext ctx;
        ASSERT_TRUE(stub->Initialize(&ctx, minimalInitializeRequest(), &iresp).ok());
    }
}
```

- [ ] **Step 3: Run the tests**

Run: `bazel test //server:radar_reprojection_server_test`
Expected: PASS — five tests total now (precondition, init success, reproject success, unknown radarId, re-init).

- [ ] **Step 4: Commit**

```bash
git add server/RadarReprojectionServer.cpp server/test/RadarReprojectionServer_test.cpp
git commit -m "Implement Reproject RPC with radarId validation and re-init coverage"
```

---

## Task 11: Server entry point (`MainServer.cpp`) + binary

**Files:**
- Create: `server/MainServer.cpp`
- Modify: `server/BUILD.bazel`

- [ ] **Step 1: Write `MainServer.cpp`**

```cpp
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>

#include "server/RadarReprojectionServer.hpp"

namespace {

std::atomic<grpc::Server*> g_server{nullptr};

void onSignal(int /*sig*/) {
    grpc::Server* s = g_server.load();
    if (s) {
        s->Shutdown(std::chrono::system_clock::now() + std::chrono::seconds(10));
    }
}

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " [--address=HOST:PORT] [--max-message-mb=N] [--num-threads=N]\n"
        << "  --address         default 0.0.0.0:50051\n"
        << "  --max-message-mb  default 256\n"
        << "  --num-threads     default unset (grpc default)\n";
}

bool parseInt(std::string_view s, int& out) {
    try { out = std::stoi(std::string{s}); return true; }
    catch (...) { return false; }
}

} // namespace

int main(int argc, char** argv) {
    std::string address     = "0.0.0.0:50051";
    int         maxMessageMb = 256;
    int         numThreads   = -1;

    for (int i = 1; i < argc; ++i) {
        std::string_view a = argv[i];
        if (a == "--help" || a == "-h") { printUsage(argv[0]); return 0; }
        constexpr std::string_view kAddr = "--address=";
        constexpr std::string_view kMmb  = "--max-message-mb=";
        constexpr std::string_view kNt   = "--num-threads=";
        if (a.starts_with(kAddr))      { address = std::string{a.substr(kAddr.size())}; continue; }
        if (a.starts_with(kMmb)) {
            if (!parseInt(a.substr(kMmb.size()), maxMessageMb)) {
                std::cerr << "bad --max-message-mb\n"; return 1;
            }
            continue;
        }
        if (a.starts_with(kNt)) {
            if (!parseInt(a.substr(kNt.size()), numThreads)) {
                std::cerr << "bad --num-threads\n"; return 1;
            }
            continue;
        }
        std::cerr << "unknown flag: " << a << "\n"; printUsage(argv[0]); return 1;
    }

    radar_reprojection::server::RadarReprojectionServer service;

    grpc::ServerBuilder builder;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    builder.SetMaxReceiveMessageSize(maxMessageMb << 20);
    builder.SetMaxSendMessageSize(maxMessageMb << 20);
    if (numThreads > 0) {
        builder.SetSyncServerOption(grpc::ServerBuilder::SyncServerOption::NUM_CQS,
                                    numThreads);
    }

    auto server = builder.BuildAndStart();
    if (!server) {
        std::cerr << "failed to start server on " << address << "\n";
        return 1;
    }
    g_server.store(server.get());
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    std::cerr << "RadarReprojectionServer listening on " << address
              << " (max-message-mb=" << maxMessageMb << ")\n";
    server->Wait();
    std::cerr << "RadarReprojectionServer shut down cleanly\n";
    return 0;
}
```

- [ ] **Step 2: Add the cc_binary target**

Append to `server/BUILD.bazel`:

```python
cc_binary(
    name = "radar_reprojection_server",
    srcs = ["MainServer.cpp"],
    deps = [
        ":radar_reprojection_server_lib",
        "@grpc//:grpc++",
    ],
    copts = COMMON_COPTS,
    visibility = ["//visibility:public"],
)
```

- [ ] **Step 3: Build the binary**

Run: `bazel build //server:radar_reprojection_server`
Expected: green build.

- [ ] **Step 4: Smoke-launch and ctrl-C**

Run: `bazel run //server:radar_reprojection_server -- --address=0.0.0.0:50051 &`
Then: `kill -INT %1 && wait %1`
Expected: `listening on 0.0.0.0:50051` followed by `shut down cleanly`.

- [ ] **Step 5: Commit**

```bash
git add server/MainServer.cpp server/BUILD.bazel
git commit -m "Server entry point with CLI flags and signal-driven shutdown"
```

---

## Task 12: Python smoke client

**Files:**
- Create: `examples/python/smoke_client.py`
- Create: `examples/python/BUILD.bazel`

- [ ] **Step 1: Write the smoke client**

`examples/python/smoke_client.py`:

```python
"""Smoke client: build a tiny LogData, Initialize the server, then Reproject.

Run after starting the server:
    bazel run //server:radar_reprojection_server &
    bazel run //examples/python:smoke_client
"""

import argparse
import sys

import grpc
from proto import radar_reprojector_pb2 as pb
from proto import radar_reprojector_pb2_grpc as pb_grpc


def identity_pose() -> pb.Pose:
    p = pb.Pose()
    p.rotation.w = 1.0
    return p


def fill_intrinsics(intr: pb.RadarIntrinsics) -> None:
    intr.min_range_m = 0.1
    intr.max_range_m = 100.0
    intr.half_hfov_rad = 1.0
    intr.half_vfov_rad = 0.5
    intr.hfov_res_rad = 0.01
    intr.vfov_res_rad = 0.01
    intr.range_res_m = 0.1


def build_init_request() -> pb.InitializeRequest:
    req = pb.InitializeRequest()
    req.param.use_log_intrinsics = True
    req.param.do_down_sampling = False
    req.param.max_search_size = 1
    req.param.dump = False
    req.param.bbox_scale_factor.x = 1.0
    req.param.bbox_scale_factor.y = 1.0
    req.param.bbox_scale_factor.z = 1.0

    rec = pb.LogRecording()
    rec.param.id = "r0"
    rec.param.max_num_returns = 8
    fill_intrinsics(rec.param.intrinsics)
    rec.param.to_ego.CopyFrom(identity_pose())
    scan = rec.scans.add()
    scan.timestamp_us = 0
    req.log_data.log_radar_data["r0"].CopyFrom(rec)

    ego_state = req.log_data.log_actor_data.ego_trajectory.states.add()
    ego_state.timestamp_us = 0
    ego_state.to_world.CopyFrom(identity_pose())
    ego_state.from_world.CopyFrom(identity_pose())
    ego_state.bbox_half_size_m.x = 2.0
    ego_state.bbox_half_size_m.y = 1.0
    ego_state.bbox_half_size_m.z = 0.5
    return req


def build_reproject_request() -> pb.ReprojectRequest:
    req = pb.ReprojectRequest()
    req.timestamp_us = 0
    req.radar_param.id = "r0"
    req.radar_param.max_num_returns = 8
    fill_intrinsics(req.radar_param.intrinsics)
    req.radar_param.to_ego.CopyFrom(identity_pose())
    req.ego_state.timestamp_us = 0
    req.ego_state.to_world.CopyFrom(identity_pose())
    req.ego_state.from_world.CopyFrom(identity_pose())
    req.ego_state.bbox_half_size_m.x = 2.0
    req.ego_state.bbox_half_size_m.y = 1.0
    req.ego_state.bbox_half_size_m.z = 0.5
    return req


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--address", default="localhost:50051")
    args = p.parse_args()

    channel_options = [
        ("grpc.max_receive_message_length", 256 * 1024 * 1024),
        ("grpc.max_send_message_length",    256 * 1024 * 1024),
    ]
    with grpc.insecure_channel(args.address, options=channel_options) as ch:
        stub = pb_grpc.RadarReprojectorServiceStub(ch)
        stub.Initialize(build_init_request())
        resp = stub.Reproject(build_reproject_request())
        print(f"Reproject returned {len(resp.scan.detections)} detections")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Author `examples/python/BUILD.bazel`**

```python
load("@rules_python//python:defs.bzl", "py_binary")
load("@pip//:requirements.bzl", "requirement")

py_binary(
    name = "smoke_client",
    srcs = ["smoke_client.py"],
    deps = [
        "//proto:radar_reprojector_py_proto",
        "//proto:radar_reprojector_py_grpc",
        requirement("grpcio"),
    ],
    main = "smoke_client.py",
    visibility = ["//visibility:public"],
)
```

- [ ] **Step 3: Build it**

Run: `bazel build //examples/python:smoke_client`
Expected: green build.

- [ ] **Step 4: End-to-end run**

In one terminal: `bazel run //server:radar_reprojection_server`
In another: `bazel run //examples/python:smoke_client`
Expected: client prints `Reproject returned N detections` (N may be 0 — we are not validating algorithm correctness, only the wire). Server stays up.

- [ ] **Step 5: Commit**

```bash
git add examples/python/smoke_client.py examples/python/BUILD.bazel
git commit -m "Python smoke client: end-to-end Initialize + Reproject"
```

---

## Task 13: Remove `RadarReprojectionManager`

**Files:**
- Delete: `RadarReprojectionManager.hpp`
- Delete: `RadarReprojectionManager.cpp`
- Delete: `dataset/test/RadarReprojectionManager_test.cpp`
- Modify: `BUILD.bazel` (top level)
- Modify: `dataset/BUILD.bazel`
- Modify: `dataset/RadarScan.hpp`

- [ ] **Step 1: Delete the source files**

Run:
```bash
git rm RadarReprojectionManager.hpp RadarReprojectionManager.cpp \
       dataset/test/RadarReprojectionManager_test.cpp
```

- [ ] **Step 2: Drop `radar_reprojection_manager` from top-level `BUILD.bazel`**

Open `BUILD.bazel`. Delete the entire `cc_library(name = "radar_reprojection_manager", ...)` block (currently around lines 22-34).

- [ ] **Step 3: Drop the test target from `dataset/BUILD.bazel`**

Open `dataset/BUILD.bazel`. Delete the `cc_test(name = "radar_reprojection_manager_test", ...)` block (currently around lines 239-247).

- [ ] **Step 4: Update the stale comment in `dataset/RadarScan.hpp`**

Change the line that reads roughly:

```cpp
// parquet. Public output of RadarReprojectionManager::getRadarScan().
```

to:

```cpp
// parquet. Historically the public output of RadarReprojectionManager;
// no longer used by any binary in this repo (kept for in-tree consumers
// that haven't migrated to the gRPC server).
```

- [ ] **Step 5: Verify the workspace still builds and all tests pass**

Run: `bazel build //... && bazel test //...`
Expected: green. There should be no remaining reference to `RadarReprojectionManager` anywhere.

Run a sanity grep:
```bash
grep -rn "RadarReprojectionManager" --include="*.hpp" --include="*.cpp" \
       --include="*.bazel" --include="*.bzl" --include="*.md" \
       /home/yahlu/nv_ws/radar_reprojection 2>/dev/null | grep -v bazel-
```
Expected: zero hits (other than this plan and the design spec, which are documentation about the removal).

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "Remove RadarReprojectionManager; superseded by gRPC server"
```

---

## Task 14: Update README and final integration check

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Read the current README**

Run: `cat README.md` — note the existing structure so you can preserve heading style.

- [ ] **Step 2: Replace the `RadarReprojectionManager` section**

The README currently has a section starting at the line `### Building \`RadarReprojectionManager.cpp\`` (around line 53) and continues through the example invocations (around line 85). Replace that whole section with:

```markdown
### Running the gRPC server

`server/MainServer.cpp` produces a standalone gRPC server that exposes
`algorithm::RadarReprojector` over the wire. Build and run from the
repo root:

```bash
bazel build //server:radar_reprojection_server
bazel run   //server:radar_reprojection_server -- --address=0.0.0.0:50051
```

Optimized build:

```bash
bazel build -c opt //server:radar_reprojection_server
```

Debug build:

```bash
bazel build --config=debug //server:radar_reprojection_server
```

Run all tests (algorithm + server + dataset):

```bash
bazel test //...
```

A trivial Python smoke client is at `examples/python/smoke_client.py`:

```bash
bazel run //examples/python:smoke_client -- --address=localhost:50051
```
```

- [ ] **Step 3: Whole-tree build + test**

Run:
```bash
bazel build //...
bazel test  //...
```
Expected: green. Specifically:
- `//algorithm:basic_types_test` passes (unchanged).
- `//server:conversions_test` passes (15 tests after Tasks 3-7).
- `//server:radar_reprojection_server_test` passes (5 tests after Tasks 8-10).
- All `//dataset:*` tests still pass (RRM test removed in Task 13).

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "README: replace RRM section with gRPC server quick-start"
```

- [ ] **Step 5: Final review**

Run: `git log --oneline -16`
Expected: 14 commits since the design-doc commit (`ba363f1`), one per task. `git status` should be clean.

---

## Self-Review Notes

1. **Spec coverage check.**
   - §1 (scope, repo delta) ↔ Tasks 1, 11, 13, 14.
   - §2 (wire format, message-size budget) ↔ Tasks 1 (proto edit), 11 (server side), 12 (client side).
   - §3 (module layout, build graph) ↔ Tasks 2, 3, 8, 11, 12.
   - §4 (server class, lifecycle, locking) ↔ Tasks 8, 9, 10.
   - §5 (conversions; `from_world` recompute) ↔ Tasks 3-7. Task 4 has a dedicated test for the `from_world` invariant.
   - §6 (entry point, CLI flags, signals) ↔ Task 11.
   - §7 (Python smoke client) ↔ Task 12.
   - §8 (error model: gRPC Status; empty `InitializeResponse`) ↔ Tasks 1 (proto edit), 9, 10.
   - §9 (test plan) ↔ Tasks 3-10 (unit tests) and Task 12 (manual integration).
   - §10 (RRM removal, comment update) ↔ Task 13.
   - §11 (risks: dep conflicts, message-size cap, dataset orphan, no TLS) — engineer is told to react in Task 1 step 4 and Task 13 step 4. The 256MB cap and TLS exclusion live in `MainServer.cpp` (Task 11).
   - §12 (locked decisions) ↔ all of the above.

2. **Type-name consistency.** All conversion declarations in Task 3 match every later use; `m_reprojector`, `m_logData`, `m_radarParams`, `m_mutex` are spelled the same in the header (Task 8), the `Initialize` impl (Task 9), and the `Reproject` impl (Task 10).

3. **No placeholders.** Every step gives the actual content (code, command, expected output). Where a fact is acknowledged as not 100% pinned (BCR rule paths, version compatibility), the step gives a concrete remediation (run `bazel mod graph`, grep `@grpc//bazel:*`).
