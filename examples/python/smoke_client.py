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

    # State 0 — algorithm requires >= 2 ego poses.
    ego_state0 = req.log_data.log_actor_data.ego_trajectory.states.add()
    ego_state0.timestamp_us = 0
    ego_state0.to_world.CopyFrom(identity_pose())
    ego_state0.from_world.CopyFrom(identity_pose())
    ego_state0.bbox_half_size_m.x = 2.0
    ego_state0.bbox_half_size_m.y = 1.0
    ego_state0.bbox_half_size_m.z = 0.5

    # State 1 — second pose required by the trajectory validator.
    ego_state1 = req.log_data.log_actor_data.ego_trajectory.states.add()
    ego_state1.timestamp_us = 100000  # 100 ms later
    ego_state1.to_world.CopyFrom(identity_pose())
    ego_state1.from_world.CopyFrom(identity_pose())
    ego_state1.bbox_half_size_m.x = 2.0
    ego_state1.bbox_half_size_m.y = 1.0
    ego_state1.bbox_half_size_m.z = 0.5

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
