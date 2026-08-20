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

#ifndef RADAR_REPROJECTION_ALGORITHM_RADAR_REPROJECTOR_HPP_
#define RADAR_REPROJECTION_ALGORITHM_RADAR_REPROJECTOR_HPP_

#include <memory>
#include <map>

#include "algorithm/Logger.hpp"

#include "JsonIO.hpp"
#include "LogTypes.hpp"
#include "ReprojectorTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

/**
 * @class RadarReprojector
 * @brief the algorithm class for reprojecting radar data
 *
 * This class pre-process log data and then can reproject the radar scan at a new perspective during simulation.
 */
class RadarReprojector
{
    constexpr static char LOG_TAG[] = "ndas_sim_light::RadarReprojector";

public:
    RadarReprojector();

    /**
     * @brief Initialize the radar reprojector and let it pre-process the log data.
     *
     * @param param parameters to control algorithm behavior
     * @param logDataPtr pointer to the log data
     * @return true if initialization was successful
     * @return false if initialization failed
     */
    bool initialize(const ReprojectorParam& param, std::shared_ptr<LogData> logDataPtr);

    /**
     * @brief Reproject to get a new radar scan from a custom perspective.
     *
     * @param input the packed input parameters including radar id, timestamp, ego pose and actor poses
     * @return RadarScan the reprojected radar scan
     */
    RadarScan reproject(const ReprojectInput& input);

private:
    bool paramIsValid() const;
    bool logDataIsValid() const;
    void computeIntrinsicsFromLog();
    void genInitializeContext();
    void genInitializeFrameContext(const std::string& radarId, size_t scanIndex);
    void genInitializeFramePointclouds(const std::string& radarId, size_t scanIndex);
    void genReprojectContext(const ReprojectInput& input);
    void reprojectDetections();
    void reprojectStaticDetections();
    std::vector<ViewIndex> genCommonBfsSeeds(const RadarView& newRadarView);
    void reprojectActorDetections(const std::string& actorId);
    ViewIndex findBestStaticScanForTarget(const RadarView& newRadarView, const Eigen::Vector2f& target, const std::vector<ViewIndex>& seeds) const;
    ViewIndex findBestActorScan(const RadarView& newRadarView, const std::string& actorId) const;
    void downsampleDetections();

    ReprojectorParam m_param;
    std::shared_ptr<LogData> m_logDataPtr;
    std::shared_ptr<ReprojectorJsonDumper> m_jsonDumper;
    // initialization
    InitializeContext m_initializeContext;
    std::map<std::string, RadarIntrinsics> m_logRadarIntrinsics;
    GridLod<size_t> m_staticGridLod;
    // reprojection
    ReprojectContext m_reprojectContext;
};

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_RADAR_REPROJECTOR_HPP_
