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

#ifndef RADAR_REPROJECTION_SERVER_RADAR_REPROJECTION_SERVER_HPP_
#define RADAR_REPROJECTION_SERVER_RADAR_REPROJECTION_SERVER_HPP_

#include <memory>
#include <mutex>
#include <unordered_map>

#include <grpcpp/grpcpp.h>

#include "algorithm/LogTypes.hpp"
#include "algorithm/RadarReprojector.hpp"
#include "algorithm/RadarTypes.hpp"

#include "proto/radar_reprojector.grpc.pb.h"

namespace radar_reprojection::server {

class RadarReprojectionServer final
    : public proto::RadarReprojectorService::Service {
public:
    RadarReprojectionServer();

    grpc::Status Initialize(grpc::ServerContext* ctx,
                            const proto::InitializeRequest* req,
                            proto::InitializeResponse* resp) override;

    grpc::Status Reproject(grpc::ServerContext* ctx,
                           const proto::ReprojectRequest* req,
                           proto::ReprojectResponse* resp) override;

private:
    std::mutex                                                m_mutex;
    std::unique_ptr<algorithm::RadarReprojector>              m_reprojector;
    std::shared_ptr<algorithm::LogData>                       m_logData;
    std::unordered_map<std::string, algorithm::RadarParam>    m_radarParams;
};

} // namespace radar_reprojection::server

#endif // RADAR_REPROJECTION_SERVER_RADAR_REPROJECTION_SERVER_HPP_
