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

#ifndef RADAR_REPROJECTION_ALGORITHM_REPROJECTOR_TYPES_HPP_
#define RADAR_REPROJECTION_ALGORITHM_REPROJECTOR_TYPES_HPP_

#include <map>
#include <string>

#include <Eigen/Dense>

#include "algorithm/BasicTypes.hpp"

#include "ActorTypes.hpp"
#include "GridLod.hpp"
#include "RadarTypes.hpp"
#include "RadarView.hpp"

namespace radar_reprojection
{
namespace algorithm
{

// parameters for the algorithm
struct ReprojectorParam
{
    bool useLogIntrinsics;           // compute intrinsics from log data
    bool doDownSampling;             // whether to down sample detections based on resolution
    Eigen::Vector3f bboxScaleFactor; // scale bounding box to incorporate noise
    Eigen::Vector3f bboxMargin;      // add margin to incorporate noise
    size_t maxSearchSize;            // max candidate scans scored per cell. 30 is usually enough. set to 1 to mimic v1 behavior.
    bool dump;
    std::string dumpFolder;
};

/**
 * @brief input parameters for calling the reproject() function in simulation
 */
struct ReprojectInput
{
    dwTime_t timestampUs; // in simulation domain
    RadarParam radarParam;
    ActorState egoState;
    std::map<std::string, ActorState> actorToState;
};

using ViewIndex = std::tuple<std::string, size_t>;

struct RadarPoint
{
    Eigen::Vector3f positionM;
    Eigen::Vector3f velocityMps;
    float32_t rcsDbm;
    ReferenceIndex reference;
};

/**
 * @brief static pointcloud extracted from 1 log radar scan
 */
struct StaticPointcloud
{
    Eigen::Affine3f sensorToWorld;
    RadarView view;                           // in world frame
    GridLod<std::vector<RadarPoint>> gridLod; // in world frame
};

/**
 * @brief 1 actor pointcloud extracted from 1 log radar scan
 */
struct ActorPointcloud
{
    std::string actorId;
    Eigen::Affine3f sensorToActor;
    RadarView view;                 // in actor's local frame
    std::vector<RadarPoint> points; // in actor's local frame
};

struct InitializeFrameContext
{
    std::string radarId;
    size_t scanIndex;
    dwTime_t timestampUs;
    ActorState egoState;
    std::map<std::string, ActorState> actorToState;
    StaticPointcloud staticPointcloud;
    std::map<std::string, ActorPointcloud> actorToPointcloud;
};

using InitializeContext = std::map<std::string, std::vector<InitializeFrameContext>>;

struct ReprojectContext
{
    ReprojectInput input;
    Eigen::Affine3f sensorToWorld;
    Eigen::Affine3f worldToSensor;
    std::vector<std::tuple<Eigen::Vector2f, Eigen::Vector2f>> visitedCells;
    RadarScan unfilteredScan;
    RadarScan filteredScan;
};

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_REPROJECTOR_TYPES_HPP_
