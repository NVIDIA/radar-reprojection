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
#include "server/Conversions.hpp"

namespace radar_reprojection::server {

RadarReprojectionServer::RadarReprojectionServer() = default;

grpc::Status RadarReprojectionServer::Initialize(
    grpc::ServerContext* /*ctx*/,
    const proto::InitializeRequest* req,
    proto::InitializeResponse* /*resp*/) {
    std::lock_guard<std::mutex> lock(m_mutex);
    try {
        algorithm::ReprojectorParam param = fromProto(req->param());
        auto logData = std::make_shared<algorithm::LogData>(
            fromProto(req->log_data()));

        auto reprojector = std::make_unique<algorithm::RadarReprojector>();
        if (!reprojector->initialize(param, logData)) {
            return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                                "RadarReprojector::initialize returned false");
        }

        std::unordered_map<std::string, algorithm::RadarParam> radarParams;
        for (const auto& [radarId, recording] : logData->logRadarData) {
            radarParams.emplace(radarId, recording.param);
        }

        m_reprojector  = std::move(reprojector);
        m_logData      = std::move(logData);
        m_radarParams  = std::move(radarParams);
        return grpc::Status::OK;
    } catch (const std::exception& e) {
        return grpc::Status(grpc::StatusCode::INTERNAL, e.what());
    }
}

grpc::Status RadarReprojectionServer::Reproject(
    grpc::ServerContext* /*ctx*/,
    const proto::ReprojectRequest* req,
    proto::ReprojectResponse* resp) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_reprojector) {
        return grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                            "Reproject called before Initialize");
    }
    if (m_radarParams.find(req->radar_param().id()) == m_radarParams.end()) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
            "unknown radarId: " + req->radar_param().id());
    }
    try {
        algorithm::ReprojectInput input = fromProto(*req);
        algorithm::RadarScan scan = m_reprojector->reproject(input);
        *resp->mutable_scan() = toProto(scan);
        return grpc::Status::OK;
    } catch (const std::exception& e) {
        return grpc::Status(grpc::StatusCode::INTERNAL, e.what());
    }
}

} // namespace radar_reprojection::server
