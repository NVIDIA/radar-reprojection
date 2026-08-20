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

#include "server/Conversions.hpp"

namespace radar_reprojection::server {

Eigen::Vector3f fromProto(const proto::Vec3f& p) {
    return Eigen::Vector3f{p.x(), p.y(), p.z()};
}

proto::Vec3f toProto(const Eigen::Vector3f& v) {
    proto::Vec3f p;
    p.set_x(v.x());
    p.set_y(v.y());
    p.set_z(v.z());
    return p;
}

Eigen::Affine3f fromProto(const proto::Pose& p) {
    Eigen::Affine3f t = Eigen::Affine3f::Identity();
    t.translate(Eigen::Vector3f{p.translation().x(),
                                p.translation().y(),
                                p.translation().z()});
    t.rotate(Eigen::Quaternionf{p.rotation().w(),
                                p.rotation().x(),
                                p.rotation().y(),
                                p.rotation().z()});
    return t;
}

proto::Pose toProto(const Eigen::Affine3f& a) {
    // Affine3f assumed to be a rigid transform; scaling/shearing is dropped.
    Eigen::Quaternionf q{a.linear()};
    proto::Pose p;
    p.mutable_translation()->set_x(a.translation().x());
    p.mutable_translation()->set_y(a.translation().y());
    p.mutable_translation()->set_z(a.translation().z());
    p.mutable_rotation()->set_x(q.x());
    p.mutable_rotation()->set_y(q.y());
    p.mutable_rotation()->set_z(q.z());
    p.mutable_rotation()->set_w(q.w());
    return p;
}

algorithm::ActorState fromProto(const proto::ActorState& p) {
    algorithm::ActorState s{};
    s.timestampUs = p.timestamp_us();
    s.toWorld     = fromProto(p.to_world());
    // Recompute from_world so clients cannot provide inconsistent inverse transforms.
    s.fromWorld   = s.toWorld.inverse();
    s.velocityMps   = fromProto(p.velocity_mps());
    s.bboxHalfSizeM = fromProto(p.bbox_half_size_m());
    s.bboxCenterM   = fromProto(p.bbox_center_m());
    return s;
}

proto::ActorState toProto(const algorithm::ActorState& s) {
    proto::ActorState p;
    p.set_timestamp_us(s.timestampUs);
    *p.mutable_to_world()         = toProto(s.toWorld);
    *p.mutable_from_world()       = toProto(s.fromWorld);
    *p.mutable_velocity_mps()     = toProto(s.velocityMps);
    *p.mutable_bbox_half_size_m() = toProto(s.bboxHalfSizeM);
    *p.mutable_bbox_center_m()    = toProto(s.bboxCenterM);
    return p;
}

algorithm::RadarIntrinsics fromProto(const proto::RadarIntrinsics& p) {
    algorithm::RadarIntrinsics s{};
    s.minRangeM   = p.min_range_m();
    s.maxRangeM   = p.max_range_m();
    s.halfHfovRad = p.half_hfov_rad();
    s.halfVfovRad = p.half_vfov_rad();
    s.hfovResRad  = p.hfov_res_rad();
    s.vfovResRad  = p.vfov_res_rad();
    s.rangeResM   = p.range_res_m();
    return s;
}

proto::RadarIntrinsics toProto(const algorithm::RadarIntrinsics& s) {
    proto::RadarIntrinsics p;
    p.set_min_range_m(s.minRangeM);
    p.set_max_range_m(s.maxRangeM);
    p.set_half_hfov_rad(s.halfHfovRad);
    p.set_half_vfov_rad(s.halfVfovRad);
    p.set_hfov_res_rad(s.hfovResRad);
    p.set_vfov_res_rad(s.vfovResRad);
    p.set_range_res_m(s.rangeResM);
    return p;
}

algorithm::RadarParam fromProto(const proto::RadarParam& p) {
    algorithm::RadarParam s{};
    s.id            = p.id();
    s.intrinsics    = fromProto(p.intrinsics());
    s.toEgo         = fromProto(p.to_ego());
    s.maxNumReturns = p.max_num_returns();
    return s;
}

proto::RadarParam toProto(const algorithm::RadarParam& s) {
    proto::RadarParam p;
    p.set_id(s.id);
    *p.mutable_intrinsics() = toProto(s.intrinsics);
    *p.mutable_to_ego()     = toProto(s.toEgo);
    p.set_max_num_returns(s.maxNumReturns);
    return p;
}

algorithm::ReferenceIndex fromProto(const proto::ReferenceIndex& p) {
    return algorithm::ReferenceIndex{
        p.radar_id(), p.scan_index(), p.detection_index(), p.actor_id()};
}

proto::ReferenceIndex toProto(const algorithm::ReferenceIndex& s) {
    proto::ReferenceIndex p;
    p.set_radar_id(s.radarId);
    p.set_scan_index(s.scanIndex);
    p.set_detection_index(s.detectionIndex);
    p.set_actor_id(s.actorId);
    return p;
}

algorithm::RadarDetection fromProto(const proto::RadarDetection& p) {
    algorithm::RadarDetection s{};
    s.azimuthRad        = p.azimuth_rad();
    s.elevationRad      = p.elevation_rad();
    s.distanceM         = p.distance_m();
    s.radialVelocityMps = p.radial_velocity_mps();
    s.rcsDbm            = p.rcs_dbm();
    s.reference         = fromProto(p.reference());
    s.velInWorld        = fromProto(p.vel_in_world());
    return s;
}

proto::RadarDetection toProto(const algorithm::RadarDetection& s) {
    proto::RadarDetection p;
    p.set_azimuth_rad(s.azimuthRad);
    p.set_elevation_rad(s.elevationRad);
    p.set_distance_m(s.distanceM);
    p.set_radial_velocity_mps(s.radialVelocityMps);
    p.set_rcs_dbm(s.rcsDbm);
    *p.mutable_reference()    = toProto(s.reference);
    *p.mutable_vel_in_world() = toProto(s.velInWorld);
    return p;
}

algorithm::RadarScan fromProto(const proto::RadarScan& p) {
    algorithm::RadarScan s{};
    s.timestampUs = p.timestamp_us();
    s.detections.reserve(p.detections_size());
    for (const auto& d : p.detections()) {
        s.detections.push_back(fromProto(d));
    }
    return s;
}

proto::RadarScan toProto(const algorithm::RadarScan& s) {
    proto::RadarScan p;
    p.set_timestamp_us(s.timestampUs);
    for (const auto& d : s.detections) {
        *p.add_detections() = toProto(d);
    }
    return p;
}

algorithm::LogRecording fromProto(const proto::LogRecording& p) {
    algorithm::LogRecording s{};
    s.param = fromProto(p.param());
    s.scans.reserve(p.scans_size());
    for (const auto& sc : p.scans()) {
        s.scans.push_back(fromProto(sc));
    }
    return s;
}

proto::LogRecording toProto(const algorithm::LogRecording& s) {
    proto::LogRecording p;
    *p.mutable_param() = toProto(s.param);
    for (const auto& sc : s.scans) {
        *p.add_scans() = toProto(sc);
    }
    return p;
}

algorithm::LogActorData fromProto(const proto::LogActorData& p) {
    algorithm::LogActorData s{};
    s.egoTrajectory.reserve(p.ego_trajectory().states_size());
    for (const auto& st : p.ego_trajectory().states()) {
        s.egoTrajectory.push_back(fromProto(st));
    }
    for (const auto& [actorId, traj] : p.actor_to_trajectory()) {
        algorithm::ActorTrajectory out;
        out.reserve(traj.states_size());
        for (const auto& st : traj.states()) {
            out.push_back(fromProto(st));
        }
        s.actorToTrajectory.emplace(actorId, std::move(out));
    }
    return s;
}

proto::LogActorData toProto(const algorithm::LogActorData& s) {
    proto::LogActorData p;
    auto* ego = p.mutable_ego_trajectory();
    for (const auto& st : s.egoTrajectory) {
        *ego->add_states() = toProto(st);
    }
    auto& outMap = *p.mutable_actor_to_trajectory();
    for (const auto& [actorId, traj] : s.actorToTrajectory) {
        proto::ActorTrajectory pt;
        for (const auto& st : traj) {
            *pt.add_states() = toProto(st);
        }
        outMap[actorId] = std::move(pt);
    }
    return p;
}

algorithm::LogData fromProto(const proto::LogData& p) {
    algorithm::LogData s{};
    for (const auto& [radarId, rec] : p.log_radar_data()) {
        s.logRadarData.emplace(radarId, fromProto(rec));
    }
    s.logActorData = fromProto(p.log_actor_data());
    return s;
}

proto::LogData toProto(const algorithm::LogData& s) {
    proto::LogData p;
    auto& outMap = *p.mutable_log_radar_data();
    for (const auto& [radarId, rec] : s.logRadarData) {
        outMap[radarId] = toProto(rec);
    }
    *p.mutable_log_actor_data() = toProto(s.logActorData);
    return p;
}

algorithm::ReprojectorParam fromProto(const proto::ReprojectorParam& p) {
    algorithm::ReprojectorParam s{};
    s.useLogIntrinsics = p.use_log_intrinsics();
    s.doDownSampling   = p.do_down_sampling();
    s.bboxScaleFactor  = fromProto(p.bbox_scale_factor());
    s.bboxMargin       = fromProto(p.bbox_margin());
    s.maxSearchSize    = static_cast<size_t>(p.max_search_size());
    s.dump             = p.dump();
    s.dumpFolder       = p.dump_folder();
    return s;
}

proto::ReprojectorParam toProto(const algorithm::ReprojectorParam& s) {
    proto::ReprojectorParam p;
    p.set_use_log_intrinsics(s.useLogIntrinsics);
    p.set_do_down_sampling(s.doDownSampling);
    *p.mutable_bbox_scale_factor() = toProto(s.bboxScaleFactor);
    *p.mutable_bbox_margin()       = toProto(s.bboxMargin);
    p.set_max_search_size(static_cast<uint64_t>(s.maxSearchSize));
    p.set_dump(s.dump);
    p.set_dump_folder(s.dumpFolder);
    return p;
}

algorithm::ReprojectInput fromProto(const proto::ReprojectRequest& p) {
    algorithm::ReprojectInput in{};
    in.timestampUs = p.timestamp_us();
    in.radarParam  = fromProto(p.radar_param());
    in.egoState    = fromProto(p.ego_state());
    for (const auto& [actorId, st] : p.actor_to_state()) {
        in.actorToState.emplace(actorId, fromProto(st));
    }
    return in;
}

} // namespace radar_reprojection::server
