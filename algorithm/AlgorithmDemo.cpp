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

#include <Eigen/Dense>

#include "algorithm/BasicTypes.hpp"
#include "algorithm/Logger.hpp"

#include "JsonIO.hpp"
#include "RadarReprojector.hpp"
#include "TransformUtilities.hpp"

using namespace radar_reprojection::algorithm;

static constexpr char LOG_TAG[] = "ndas_sim_light::radarreprojection::algorithm_demo";
const std::string RADAR_ID      = "radar:front:center";

ReprojectInput genReprojectInput(
    const std::shared_ptr<LogData>& logDataPtr,
    dwTime_t timestampUs,
    float32_t egoLateralTranslationM = 0.0f,
    float32_t egoRotationZ           = 0.0f)
{
    ReprojectInput input;
    input.timestampUs = timestampUs;
    input.radarParam  = logDataPtr->logRadarData.at(RADAR_ID).param;
    input.egoState    = slerp(logDataPtr->logActorData.egoTrajectory, timestampUs);
    // (optional) Add divergence to ego (e.g. 3m, 30 deg).
    input.egoState.toWorld.translation() += input.egoState.toWorld.linear().col(1) * egoLateralTranslationM;
    input.egoState.toWorld.rotate(Eigen::AngleAxisf(egoRotationZ, Eigen::Vector3f::UnitZ()));

    input.actorToState.clear();
    for (auto & [ id, trajectory ] : logDataPtr->logActorData.actorToTrajectory)
    {
        if (timestampUs < trajectory.front().timestampUs || timestampUs >= trajectory.back().timestampUs)
        {
            continue;
        }
        input.actorToState[id] = slerp(trajectory, timestampUs);
    }
    return input;
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        DW_LOGE << "Usage: radar_reproj_algo_test <log_data_json_path> <dump_folder_path> <reproj_frame_index>" << dw_log::endl;
        return 1;
    }

    // parse arguments
    std::string logDataJsonPath = argv[1];
    std::string dumpFolderPath  = argv[2];
    if (dumpFolderPath.back() != '/')
    {
        dumpFolderPath.push_back('/');
    }
    size_t reprojFrameIndex = std::stoul(argv[3]);

    // reprojector algorithm parameters
    ReprojectorParam param;
    param.useLogIntrinsics = true;
    param.doDownSampling   = false;
    param.bboxScaleFactor  = Eigen::Vector3f(1.25f, 1.25f, 2.0f);
    param.bboxMargin       = Eigen::Vector3f(0.25f, 0.25f, 1.0f);
    param.maxSearchSize    = 30;
    param.dump             = true;
    param.dumpFolder       = dumpFolderPath;

    // load log data
    LogDataLoader loader(logDataJsonPath);
    auto logDataPtr = loader.get();

    // initialize reprojector
    RadarReprojector rr;
    rr.initialize(param, logDataPtr);

    auto start = std::chrono::high_resolution_clock::now();

    auto reprojInput = genReprojectInput(
        logDataPtr,
        logDataPtr->logRadarData.at(RADAR_ID).scans[reprojFrameIndex].timestampUs,
        3.0f,
        -0.5f);
    auto scan = rr.reproject(reprojInput);
    DW_LOGI << "reprojected " << scan.detections.size() << " detections." << dw_log::endl;

    auto end           = std::chrono::high_resolution_clock::now();
    int64_t durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    DW_LOGI << "Time taken: " << durationMs << " ms" << dw_log::endl;

    std::map<ViewIndex, size_t> refCount;
    for (const auto& detection : scan.detections)
    {
        if (detection.reference.actorId != "")
        {
            continue;
        }
        refCount[std::make_tuple(detection.reference.radarId, detection.reference.scanIndex)]++;
    }
    for (const auto & [ ref, count ] : refCount)
    {
        const auto & [ radarId, scanIndex ] = ref;
        const auto& refScan = logDataPtr->logRadarData.at(radarId).scans[scanIndex];
        DW_LOGI << radarId << "[" << scanIndex << "]: " << count << " / " << refScan.detections.size() << dw_log::endl;
    }

    return 0;
}
