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

#ifndef RADAR_REPROJECTION_ALGORITHM_ACTOR_TYPES_HPP_
#define RADAR_REPROJECTION_ALGORITHM_ACTOR_TYPES_HPP_

#include <vector>

#include <Eigen/Dense>

#include "algorithm/BasicTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

struct ActorState
{
    dwTime_t timestampUs; // must be in the same domain as ego trajectory
    Eigen::Affine3f toWorld;
    Eigen::Affine3f fromWorld;
    Eigen::Vector3f velocityMps;   // in world frame
    Eigen::Vector3f bboxHalfSizeM; // half of (length, width, height)
    Eigen::Vector3f bboxCenterM;   // position of the bbox center in actor local frame
};

using ActorTrajectory = std::vector<ActorState>;

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_ACTOR_TYPES_HPP_
