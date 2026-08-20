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

#ifndef RADAR_REPROJECTION_ALGORITHM_JSON_IO_HPP_
#define RADAR_REPROJECTION_ALGORITHM_JSON_IO_HPP_

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ActorTypes.hpp"
#include "LogTypes.hpp"
#include "RadarTypes.hpp"
#include "ReprojectorTypes.hpp"

using Json = nlohmann::json;

namespace radar_reprojection
{
namespace algorithm
{

class LogDataLoader
{
public:
    LogDataLoader(const std::string& logDataJsonPath);
    std::shared_ptr<LogData> get() const;

private:
    static void loadFromFile(Json& j, const std::string& path);
    template <typename T>
    static void loadFromJson(std::vector<T>& vector, const Json& j)
    {
        vector.clear();
        vector.reserve(j.size());
        for (const auto& elementJson : j)
        {
            T element;
            loadFromJson(element, elementJson);
            vector.push_back(element);
        }
    }
    template <typename T>
    static void loadFromJson(std::map<std::string, T>& map, const Json& j)
    {
        map.clear();
        for (auto it = j.begin(); it != j.end(); ++it)
        {
            T value;
            loadFromJson(value, it.value());
            map[it.key()] = value;
        }
    }
    static void loadFromJson(LogData& data, const Json& j);
    static void loadFromJson(LogActorData& data, const Json& j);
    static void loadFromJson(ActorState& state, const Json& j);
    static void loadFromJson(Eigen::Affine3f& affine, const Json& j);
    static void loadFromJson(Eigen::Vector3f& vector, const Json& j);
    static void loadFromJson(LogRecording& recording, const Json& j);
    static void loadFromJson(RadarParam& param, const Json& j);
    static void loadFromJson(RadarScan& scan, const Json& j);
    static void loadFromJson(RadarDetection& detection, const Json& j);
    std::shared_ptr<LogData> m_logDataPtr;
};

class ReprojectorJsonDumper
{
public:
    explicit ReprojectorJsonDumper(const std::string& dumpFolder);
    void dumpLogData(const LogData& data) const;
    void dumpInitializeResult(const InitializeContext& context) const;
    void dumpReprojectResult(const ReprojectContext& context) const;

private:
    static void dumpToFile(const std::string& path, const Json& j);
    template <typename T>
    static Json dumpToJson(const std::vector<T>& vector)
    {
        Json j = Json::array();
        for (const auto& element : vector)
        {
            j.push_back(dumpToJson(element));
        }
        return j;
    }
    template <typename T>
    static Json dumpToJson(const std::map<std::string, T>& map)
    {
        Json j = Json::object();
        for (const auto & [ key, value ] : map)
        {
            j[key] = dumpToJson(value);
        }
        return j;
    }
    template <typename T>
    static Json dumpToJson(const std::map<CellIndex, T>& map)
    {
        Json j = Json::object();
        for (const auto & [ key, value ] : map)
        {
            const auto & [ x, y ] = key;
            std::string keyString = std::to_string(x) + "_" + std::to_string(y);
            j[keyString]          = dumpToJson(value);
        }
        return j;
    }
    static Json dumpToJson(const LogData& data);
    static Json dumpToJson(const LogRecording& recording);
    static Json dumpToJson(const RadarParam& param);
    static Json dumpToJson(const Eigen::Affine3f& affine);
    static Json dumpToJson(const Eigen::Vector3f& vector);
    static Json dumpToJson(const Eigen::Vector2f& vector);
    static Json dumpToJson(const RadarScan& scan);
    static Json dumpToJson(const RadarDetection& detection);
    static Json dumpToJson(const ReferenceIndex& reference);
    static Json dumpToJson(const LogActorData& data);
    static Json dumpToJson(const ActorState& state);
    static Json dumpToJson(const InitializeFrameContext& context);
    static Json dumpToJson(const StaticPointcloud& pointcloud);
    static Json dumpToJson(const RadarView& view);
    static Json dumpToJson(const RadarPoint& point);
    static Json dumpToJson(const ActorPointcloud& pointcloud);
    static Json dumpToJson(const ReprojectContext& context);
    static Json dumpToJson(const ReprojectInput& input);
    static Json dumpToJson(const std::tuple<Eigen::Vector2f, Eigen::Vector2f>& cell);
    static Json dumpToJson(size_t index);

    std::string m_dumpFolder;
};

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_JSON_IO_HPP_
