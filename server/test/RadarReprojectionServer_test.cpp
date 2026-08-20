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

    // State 0 — algorithm requires >= 2 ego poses.
    {
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
    }

    // State 1 — second pose required by the trajectory validator.
    {
        auto* st  = ego->add_states();
        st->set_timestamp_us(100000);  // 100 ms later
        auto* tw  = st->mutable_to_world();
        tw->mutable_rotation()->set_w(1.0f);
        auto* fw  = st->mutable_from_world();
        fw->mutable_rotation()->set_w(1.0f);
        auto* vmps = st->mutable_velocity_mps(); (void)vmps;
        auto* hbox = st->mutable_bbox_half_size_m();
        hbox->set_x(2.0f); hbox->set_y(1.0f); hbox->set_z(0.5f);
        auto* cbox = st->mutable_bbox_center_m(); (void)cbox;
    }

    return req;
}

} // namespace

TEST_F(ServerFixture, ReprojectBeforeInitializeFailsPrecondition) {
    rrp::ReprojectRequest req;
    rrp::ReprojectResponse resp;
    grpc::ClientContext ctx;
    grpc::Status s = stub->Reproject(&ctx, req, &resp);
    EXPECT_EQ(s.error_code(), grpc::StatusCode::FAILED_PRECONDITION);
}

TEST_F(ServerFixture, InitializeSucceedsWithMinimalLogData) {
    rrp::InitializeRequest req = minimalInitializeRequest();
    rrp::InitializeResponse resp;
    grpc::ClientContext ctx;
    grpc::Status s = stub->Initialize(&ctx, req, &resp);
    EXPECT_TRUE(s.ok()) << s.error_code() << ": " << s.error_message();
}

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
