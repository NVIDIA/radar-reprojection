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

#include "JsonIO.hpp"

#include "algorithm/TransformUtilities.hpp"

#include <fstream>

#include "algorithm/BasicTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

// LogDataLoader

LogDataLoader::LogDataLoader(const std::string& logDataJsonPath)
{
    Json logDataJson;
    loadFromFile(logDataJson, logDataJsonPath);
    LogData logData;
    loadFromJson(logData, logDataJson);
    m_logDataPtr = std::make_shared<LogData>(std::move(logData));
}

std::shared_ptr<LogData> LogDataLoader::get() const
{
    return m_logDataPtr;
}

void LogDataLoader::loadFromFile(Json& j, const std::string& path)
{
    std::ifstream file(path);
    j = Json::parse(file);
    file.close();
}

void LogDataLoader::loadFromJson(LogData& data, const Json& j)
{
    loadFromJson(data.logActorData, j["log_actor_data"]);
    loadFromJson(data.logRadarData, j["log_radar_data"]);
}

void LogDataLoader::loadFromJson(LogActorData& data, const Json& j)
{
    loadFromJson(data.egoTrajectory, j["ego_trajectory"]);
    loadFromJson(data.actorToTrajectory, j["actor_to_trajectory"]);
}

void LogDataLoader::loadFromJson(ActorState& state, const Json& j)
{
    state.timestampUs = j["timestamp_us"].get<dwTime_t>();
    loadFromJson(state.toWorld, j["to_world"]);
    loadFromJson(state.fromWorld, j["from_world"]);
    loadFromJson(state.velocityMps, j["velocity_mps"]);
    loadFromJson(state.bboxHalfSizeM, j["bbox_half_size_m"]);
    loadFromJson(state.bboxCenterM, j["bbox_center_m"]);
}

void LogDataLoader::loadFromJson(Eigen::Affine3f& affine, const Json& j)
{
    Eigen::Matrix4f matrix;
    for (int ri = 0; ri < 4; ++ri)
    {
        for (int ci = 0; ci < 4; ++ci)
        {
            matrix(ri, ci) = j[ri * 4 + ci].get<float>();
        }
    }
    affine.matrix() = matrix;
}

void LogDataLoader::loadFromJson(Eigen::Vector3f& vector, const Json& j)
{
    vector.x() = j[0].get<float>();
    vector.y() = j[1].get<float>();
    vector.z() = j[2].get<float>();
}

void LogDataLoader::loadFromJson(LogRecording& recording, const Json& j)
{
    loadFromJson(recording.param, j["param"]);
    loadFromJson(recording.scans, j["scans"]);
}

void LogDataLoader::loadFromJson(RadarParam& param, const Json& j)
{
    param.id                     = j["id"].get<std::string>();
    param.intrinsics.minRangeM   = j["min_range_m"].get<float>();
    param.intrinsics.maxRangeM   = j["max_range_m"].get<float>();
    param.intrinsics.halfHfovRad = j["half_hfov_rad"].get<float>();
    param.intrinsics.halfVfovRad = j["half_vfov_rad"].get<float>();
    param.intrinsics.hfovResRad  = j["hfov_res_rad"].get<float>();
    param.intrinsics.vfovResRad  = j["vfov_res_rad"].get<float>();
    param.intrinsics.rangeResM   = j["range_res_m"].get<float>();
    loadFromJson(param.toEgo, j["to_ego"]);
}

void LogDataLoader::loadFromJson(RadarScan& scan, const Json& j)
{
    scan.timestampUs = j["timestamp_us"].get<dwTime_t>();
    loadFromJson(scan.detections, j["detections"]);
}

void LogDataLoader::loadFromJson(RadarDetection& detection, const Json& j)
{
    detection.azimuthRad        = j["azimuth_rad"].get<float>();
    detection.elevationRad      = j["elevation_rad"].get<float>();
    detection.distanceM         = j["distance_m"].get<float>();
    detection.radialVelocityMps = j["radial_velocity_mps"].get<float>();
    detection.rcsDbm            = j["rcs_dbm"].get<float>();
    // NOTE Do not need to load reference index from log data.
}

// ReprojectorJsonDumper

ReprojectorJsonDumper::ReprojectorJsonDumper(const std::string& dumpFolder)
{
    m_dumpFolder = dumpFolder.back() == '/' ? dumpFolder : dumpFolder + '/';
}

void ReprojectorJsonDumper::dumpLogData(const LogData& data) const
{
    Json j               = dumpToJson(data);
    std::string filename = m_dumpFolder + "log_data.json";
    dumpToFile(filename, j);
}

void ReprojectorJsonDumper::dumpInitializeResult(const InitializeContext& context) const
{
    Json j               = dumpToJson(context);
    std::string filename = m_dumpFolder + "initialize.json";
    dumpToFile(filename, j);
}

void ReprojectorJsonDumper::dumpReprojectResult(const ReprojectContext& context) const
{
    Json j               = dumpToJson(context);
    std::string filename = m_dumpFolder + "reproject_" + context.input.radarParam.id + "_" + std::to_string(context.input.timestampUs) + ".json";
    dumpToFile(filename, j);
}

void ReprojectorJsonDumper::dumpToFile(const std::string& path, const Json& j)
{
    std::ofstream file(path);
    file << j.dump(1, '\t');
    file.close();
}

Json ReprojectorJsonDumper::dumpToJson(const LogData& data)
{
    Json j;
    j["log_radar_data"] = dumpToJson(data.logRadarData);
    j["log_actor_data"] = dumpToJson(data.logActorData);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const LogRecording& recording)
{
    Json j;
    j["param"] = dumpToJson(recording.param);
    j["scans"] = dumpToJson(recording.scans);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const RadarParam& param)
{
    Json j;
    j["id"]            = param.id;
    j["min_range_m"]   = param.intrinsics.minRangeM;
    j["max_range_m"]   = param.intrinsics.maxRangeM;
    j["half_hfov_rad"] = param.intrinsics.halfHfovRad;
    j["half_vfov_rad"] = param.intrinsics.halfVfovRad;
    j["hfov_res_rad"]  = param.intrinsics.hfovResRad;
    j["vfov_res_rad"]  = param.intrinsics.vfovResRad;
    j["range_res_m"]   = param.intrinsics.rangeResM;
    j["to_ego"]        = dumpToJson(param.toEgo);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const Eigen::Vector3f& vec)
{
    Json j = Json::array();
    j.push_back(vec.x());
    j.push_back(vec.y());
    j.push_back(vec.z());
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const Eigen::Affine3f& affine)
{
    Json j = Json::array();
    for (int ri = 0; ri < 4; ++ri)
    {
        for (int ci = 0; ci < 4; ++ci)
        {
            j.push_back(affine.matrix()(ri, ci));
        }
    }
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const Eigen::Vector2f& vec)
{
    Json j = Json::array();
    j.push_back(vec.x());
    j.push_back(vec.y());
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const RadarScan& scan)
{
    Json j;
    j["timestamp_us"] = scan.timestampUs;
    j["detections"]   = dumpToJson(scan.detections);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const RadarDetection& detection)
{
    Json j;
    j["azimuth_rad"]         = detection.azimuthRad;
    j["elevation_rad"]       = detection.elevationRad;
    j["distance_m"]          = detection.distanceM;
    j["radial_velocity_mps"] = detection.radialVelocityMps;
    j["rcs_dbm"]             = detection.rcsDbm;
    j["reference"]           = dumpToJson(detection.reference);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const ReferenceIndex& reference)
{
    Json j;
    j["radar_id"]        = reference.radarId;
    j["scan_index"]      = reference.scanIndex;
    j["detection_index"] = reference.detectionIndex;
    j["actor_id"]        = reference.actorId;
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const LogActorData& data)
{
    Json j;
    j["ego_trajectory"]      = dumpToJson(data.egoTrajectory);
    j["actor_to_trajectory"] = dumpToJson(data.actorToTrajectory);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const ActorState& state)
{
    Json j;
    j["timestamp_us"]     = state.timestampUs;
    j["to_world"]         = dumpToJson(state.toWorld);
    j["from_world"]       = dumpToJson(state.fromWorld);
    j["velocity_mps"]     = dumpToJson(state.velocityMps);
    j["bbox_half_size_m"] = dumpToJson(state.bboxHalfSizeM);
    j["bbox_center_m"]    = dumpToJson(state.bboxCenterM);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const InitializeFrameContext& context)
{
    Json j;
    j["radar_id"]            = context.radarId;
    j["scan_index"]          = context.scanIndex;
    j["timestamp_us"]        = context.timestampUs;
    j["ego_state"]           = dumpToJson(context.egoState);
    j["actor_to_state"]      = dumpToJson(context.actorToState);
    j["static_pointcloud"]   = dumpToJson(context.staticPointcloud);
    j["actor_to_pointcloud"] = dumpToJson(context.actorToPointcloud);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const StaticPointcloud& pointcloud)
{
    Json j;
    j["sensor_to_world"] = dumpToJson(pointcloud.sensorToWorld);
    j["view"]            = dumpToJson(pointcloud.view);
    // TODO(keq) Consider dumping grid lod if needed.
    // j["grid_lod"]        = dumpToJson(pointcloud.gridLod);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const RadarView& view)
{
    Json j;
    j["origin_m"]      = dumpToJson(view.originM);
    j["azimuth_rad"]   = view.azimuthRad;
    j["radius_m"]      = view.radiusM;
    j["half_hfov_rad"] = view.halfHfovRad;
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const RadarPoint& point)
{
    Json j;
    j["position_m"]   = dumpToJson(point.positionM);
    j["velocity_mps"] = dumpToJson(point.velocityMps);
    j["rcs_dbm"]      = point.rcsDbm;
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const ActorPointcloud& pointcloud)
{
    Json j;
    j["actor_id"]        = pointcloud.actorId;
    j["sensor_to_actor"] = dumpToJson(pointcloud.sensorToActor);
    j["view"]            = dumpToJson(pointcloud.view);
    j["points"]          = dumpToJson(pointcloud.points);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const ReprojectContext& context)
{
    Json j;
    j["input"]           = dumpToJson(context.input);
    j["sensor_to_world"] = dumpToJson(context.sensorToWorld);
    j["world_to_sensor"] = dumpToJson(context.worldToSensor);
    j["visited_cells"]   = dumpToJson(context.visitedCells);
    j["unfiltered_scan"] = dumpToJson(context.unfilteredScan);
    j["filtered_scan"]   = dumpToJson(context.filteredScan);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const ReprojectInput& input)
{
    Json j;
    j["timestamp_us"]   = input.timestampUs;
    j["radar_param"]    = dumpToJson(input.radarParam);
    j["ego_state"]      = dumpToJson(input.egoState);
    j["actor_to_state"] = dumpToJson(input.actorToState);
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(const std::tuple<Eigen::Vector2f, Eigen::Vector2f>& cell)
{
    Json j = Json::array();
    j.push_back(dumpToJson(std::get<0>(cell)));
    j.push_back(dumpToJson(std::get<1>(cell)));
    return j;
}

Json ReprojectorJsonDumper::dumpToJson(size_t index)
{
    return Json(index);
}

} // namespace algorithm
} // namespace radar_reprojection
