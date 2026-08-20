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

#ifndef RADAR_REPROJECTION_ALGORITHM_RADAR_TYPES_HPP_
#define RADAR_REPROJECTION_ALGORITHM_RADAR_TYPES_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "algorithm/BasicTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

struct ReferenceIndex
{
    std::string radarId    = "";
    int32_t scanIndex      = -1;
    int32_t detectionIndex = -1;
    std::string actorId    = "";
};

struct RadarDetection
{
    float32_t azimuthRad;   // x+ is 0, left = +, y+ is pi/2
    float32_t elevationRad; // z+ is 0, up = +, z+ is pi/2
    float32_t distanceM;
    float32_t radialVelocityMps; // moving away from radar = +
    float32_t rcsDbm;
    ReferenceIndex reference;

    // This is for the ego pose deviation testing only.
    // It's needed to transform the velocity from the deviated pose back to the original pose.
    // Without this, the tangential velocity would be lost, introducing a velocity error during the transformation.
    Eigen::Vector3f velInWorld{Eigen::Vector3f::Zero()};
};

struct RadarScan
{
    dwTime_t timestampUs; // must be in the same domain as ego trajectory
    std::vector<RadarDetection> detections;
};

/**
 * @brief Radar intrinsic parameters
 */
struct RadarIntrinsics
{
    float32_t minRangeM;
    float32_t maxRangeM;
    float32_t halfHfovRad;
    float32_t halfVfovRad;
    float32_t hfovResRad;
    float32_t vfovResRad;
    float32_t rangeResM;
};

struct RadarParam
{
    std::string id;
    RadarIntrinsics intrinsics;
    Eigen::Affine3f toEgo{Eigen::Affine3f::Identity()};

    // Max number of returns per radar scan
    uint32_t maxNumReturns{0U};
};

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_RADAR_TYPES_HPP_
