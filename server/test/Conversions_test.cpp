// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "server/Conversions.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

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
