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
