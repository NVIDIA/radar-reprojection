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

#include "RadarReprojector.hpp"

#include <chrono>
#include <queue>
#include <set>
#include <stack>

#include "JsonIO.hpp"
#include "TransformUtilities.hpp"

namespace radar_reprojection
{
namespace algorithm
{

namespace
{

constexpr float32_t kEpsilon            = 1E-6f;
constexpr float32_t kBfsSimilarityRatio = 0.5f;
constexpr float32_t kCellSizeMax        = 20.0f;
constexpr float32_t kCellSizeMin        = 2.0f;

CellIndex getCellIndex(const Eigen::Vector2f& positionM, float32_t cellSize)
{
    return {static_cast<int32_t>(std::floor(positionM[0] / cellSize)), static_cast<int32_t>(std::floor(positionM[1] / cellSize))};
}

Eigen::Vector2f getCellCenter(const CellIndex& cellIndex, float32_t cellSize)
{
    return {(std::get<0>(cellIndex) + 0.5f) * cellSize, (std::get<1>(cellIndex) + 0.5f) * cellSize};
}

} // namespace

RadarReprojector::RadarReprojector()
{
}

bool RadarReprojector::initialize(const ReprojectorParam& param, std::shared_ptr<LogData> logDataPtr)
{
    m_param      = param;
    m_logDataPtr = logDataPtr;
    if (!paramIsValid() || !logDataIsValid())
    {
        return false;
    }
    m_jsonDumper = std::make_shared<ReprojectorJsonDumper>(param.dumpFolder);
    // Compute radar intrinsics from log data.
    if (m_param.useLogIntrinsics)
    {
        computeIntrinsicsFromLog();
    }

    m_staticGridLod.clear(kCellSizeMax, kCellSizeMin);
    genInitializeContext();

    if (m_param.dump)
    {
        m_jsonDumper->dumpLogData(*m_logDataPtr);
        m_jsonDumper->dumpInitializeResult(m_initializeContext);
    }
    return true;
}

RadarScan RadarReprojector::reproject(const ReprojectInput& input)
{
    genReprojectContext(input);
    reprojectDetections();
    if (m_param.doDownSampling)
    {
        downsampleDetections();
    }
    else
    {
        m_reprojectContext.filteredScan = m_reprojectContext.unfilteredScan; // copy
    }
    std::sort(m_reprojectContext.filteredScan.detections.begin(), m_reprojectContext.filteredScan.detections.end(), [](const auto& a, const auto& b) {
        return a.distanceM < b.distanceM;
    });
    if (m_param.dump)
    {
        m_jsonDumper->dumpReprojectResult(m_reprojectContext);
    }
    return m_reprojectContext.filteredScan;
}

bool RadarReprojector::paramIsValid() const
{
    if (m_param.maxSearchSize < 1)
    {
        DW_LOGE << "maxSearchSize must be at least 1." << dw_log::endl;
        return false;
    }
    return true;
}

bool RadarReprojector::logDataIsValid() const
{
    if (!m_logDataPtr)
    {
        DW_LOGE << "Log data is null." << dw_log::endl;
        return false;
    }
    if (m_logDataPtr->logActorData.egoTrajectory.size() < 2)
    {
        DW_LOGE << "Log ego trajectory has less than 2 poses." << dw_log::endl;
        return false;
    }
    for (const auto & [ actorId, actorTrajectory ] : m_logDataPtr->logActorData.actorToTrajectory)
    {
        if (actorTrajectory.size() < 2)
        {
            DW_LOGE << "Log actor " << actorId << "'s trajectory has less than 2 poses." << dw_log::endl;
            return false;
        }
    }
    return true;
}

void RadarReprojector::genInitializeContext()
{
    m_initializeContext.clear();
    for (const auto & [ radarId, logRecording ] : m_logDataPtr->logRadarData)
    {
        m_initializeContext[radarId].resize(logRecording.scans.size());
        for (size_t scanIndex = 0; scanIndex < logRecording.scans.size(); ++scanIndex)
        {
            genInitializeFrameContext(radarId, scanIndex);
        }
    }
}

void RadarReprojector::genInitializeFrameContext(const std::string& radarId, size_t scanIndex)
{
    const auto& logRadarScan    = m_logDataPtr->logRadarData.at(radarId).scans[scanIndex];
    const auto& radarExtrinsics = m_logDataPtr->logRadarData.at(radarId).param.toEgo;
    const auto& radarIntrinsics = m_param.useLogIntrinsics ? m_logRadarIntrinsics.at(radarId) : m_logDataPtr->logRadarData.at(radarId).param.intrinsics;
    // Create frame context object.
    auto& context       = m_initializeContext[radarId][scanIndex];
    context.radarId     = radarId;
    context.scanIndex   = scanIndex;
    context.timestampUs = logRadarScan.timestampUs;
    // Interpolate ego state at current timestamp.
    context.egoState = slerp(m_logDataPtr->logActorData.egoTrajectory, logRadarScan.timestampUs);
    // Fill static pointcloud metadata.
    auto& staticPointcloud         = context.staticPointcloud;
    staticPointcloud.sensorToWorld = context.egoState.toWorld * radarExtrinsics;
    staticPointcloud.view          = RadarView::create(staticPointcloud.sensorToWorld, radarIntrinsics);
    // Interpolate each actor's state at current timestamp.
    for (const auto & [ actorId, actorTrajectory ] : m_logDataPtr->logActorData.actorToTrajectory)
    {
        // If actor does not show up at this timestamp, skip.
        if (logRadarScan.timestampUs < actorTrajectory.front().timestampUs || logRadarScan.timestampUs > actorTrajectory.back().timestampUs)
        {
            continue;
        }
        auto& actorState      = context.actorToState[actorId];
        auto& actorPointcloud = context.actorToPointcloud[actorId];
        // Interpolate and add margin to actor's bbox.
        actorState                    = slerp(actorTrajectory, logRadarScan.timestampUs);
        actorState.bboxHalfSizeM      = actorState.bboxHalfSizeM.cwiseProduct(m_param.bboxScaleFactor) + m_param.bboxMargin;
        actorPointcloud.actorId       = actorId;
        actorPointcloud.sensorToActor = actorState.fromWorld * staticPointcloud.sensorToWorld;
        actorPointcloud.view          = RadarView::create(actorPointcloud.sensorToActor, radarIntrinsics);
    }
    genInitializeFramePointclouds(radarId, scanIndex);
}

void RadarReprojector::genInitializeFramePointclouds(const std::string& radarId, size_t scanIndex)
{
    const auto& logRadarScan = m_logDataPtr->logRadarData.at(radarId).scans[scanIndex];
    auto& context            = m_initializeContext[radarId][scanIndex];
    auto& staticPointcloud   = context.staticPointcloud;
    auto& actorToState       = context.actorToState;
    auto& actorToPointcloud  = context.actorToPointcloud;
    staticPointcloud.gridLod.clear(kCellSizeMax, kCellSizeMin);
    // Separate detections to static or dynamic.
    for (size_t detectionIndex = 0; detectionIndex < logRadarScan.detections.size(); ++detectionIndex)
    {
        const auto& detection       = logRadarScan.detections[detectionIndex];
        ReferenceIndex reference    = {radarId, static_cast<int32_t>(scanIndex), static_cast<int32_t>(detectionIndex), ""};
        auto positionInSensor       = sphericalToCartesian(detection);
        auto positionInWorld        = staticPointcloud.sensorToWorld * positionInSensor;
        auto radialDirectionInWorld = safeNormalize(positionInWorld - staticPointcloud.sensorToWorld.translation(), staticPointcloud.sensorToWorld.linear().col(0));
        // NOTE We use sensor motion to compensate and recover point's actual speed along radial direction,
        //   so that for accurate static detections, the recovered value should be close to zero.
        // TODO(keq) In theory should use sensor's velocity for projection. For now we assume ego has no angular speed and thus the same.
        float32_t speedWrtWorld = detection.radialVelocityMps + context.egoState.velocityMps.dot(radialDirectionInWorld);
        auto velocityInWorld    = radialDirectionInWorld * speedWrtWorld;
        bool isStatic           = true;
        // TODO(keq) Add spatial acceleration:
        // Option #1: Use 2D grid to hash actor bbox.
        // Option #2: Sort based on scan line direction and 2-pointer matching.
        for (const auto & [ actorId, actorState ] : actorToState)
        {
            auto positionInActor = actorState.fromWorld * positionInWorld;
            auto extent          = positionInActor - actorState.bboxCenterM;
            // Associate detection with actor if it is inside actor's bbox.
            // TODO(keq) Add check for radial velocity signature.
            if (abs(extent) < actorState.bboxHalfSizeM)
            {
                auto velocityWrtActorInWorld = velocityInWorld - actorState.velocityMps;
                auto velocityInActor         = actorState.fromWorld.linear() * velocityWrtActorInWorld;
                reference.actorId            = actorId;
                actorToPointcloud[actorId].points.push_back({positionInActor, velocityInActor, detection.rcsDbm, reference});
                isStatic = false;
                break;
            }
        }
        if (isStatic)
        {
            for (size_t level = 0; level < staticPointcloud.gridLod.numLevels(); ++level)
            {
                auto cellIndex = getCellIndex(positionInWorld.head<2>(), staticPointcloud.gridLod.cellSize(level));
                staticPointcloud.gridLod.item(level, cellIndex).push_back({positionInWorld, velocityInWorld, detection.rcsDbm, reference});
                // NOTE Might not need count. Just a boolean flag is enough.
                if (m_staticGridLod.grid(level).find(cellIndex) == m_staticGridLod.grid(level).end())
                {
                    m_staticGridLod.grid(level)[cellIndex] = 0;
                }
                ++m_staticGridLod.grid(level)[cellIndex];
            }
        }
    }
}

void RadarReprojector::genReprojectContext(const ReprojectInput& input)
{
    m_reprojectContext.input         = input;
    m_reprojectContext.sensorToWorld = input.egoState.toWorld * input.radarParam.toEgo;
    m_reprojectContext.worldToSensor = m_reprojectContext.sensorToWorld.inverse();
    m_reprojectContext.visitedCells.clear();
    m_reprojectContext.unfilteredScan.timestampUs = input.timestampUs;
    m_reprojectContext.unfilteredScan.detections.clear();
    m_reprojectContext.filteredScan.timestampUs = input.timestampUs;
    m_reprojectContext.filteredScan.detections.clear();
}

void RadarReprojector::reprojectDetections()
{
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    reprojectStaticDetections();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<float> duration     = std::chrono::duration_cast<std::chrono::duration<float>>(end - start);
    DW_LOGD << "reprojectStaticDetections time: " << duration.count() << " seconds" << dw_log::endl;

    start = std::chrono::steady_clock::now();
    for (const auto & [ actorId, actorState ] : m_reprojectContext.input.actorToState)
    {
        reprojectActorDetections(actorId);
    }
    end      = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::duration<float>>(end - start);
    DW_LOGD << "reprojectActorDetections time: " << duration.count() << " seconds" << dw_log::endl;
}

void RadarReprojector::reprojectStaticDetections()
{
    const auto& radarIntrinsics = m_param.useLogIntrinsics ? m_logRadarIntrinsics.at(m_reprojectContext.input.radarParam.id) : m_reprojectContext.input.radarParam.intrinsics;
    auto newRadarView           = RadarView::create(m_reprojectContext.sensorToWorld, radarIntrinsics);
    auto bfsSeeds               = genCommonBfsSeeds(newRadarView);
    // Obtain the top level cell indices that radar view can see.
    std::stack<CellIndexLod, std::vector<CellIndexLod>> cellIndices(std::move(newRadarView.cellIndices(kCellSizeMax)));
    // NOTE Here we are using stack and loop to emulate recursion upon divisible cells.
    while (!cellIndices.empty())
    {
        auto[level, cellIndex] = cellIndices.top();
        cellIndices.pop();
        float32_t cellSize = m_staticGridLod.cellSize(level);
        // If no log frame has points in this cell, skip.
        if (m_staticGridLod.grid(level).find(cellIndex) == m_staticGridLod.grid(level).end())
        {
            continue;
        }
        // if cell has no intersection with radar view fan shape, skip.
        if (!newRadarView.intersectWithCell(cellIndex, cellSize))
        {
            continue;
        }
        // If cell is too big, divide it into 4 sub-cells.
        auto cellCenter = getCellCenter(cellIndex, cellSize);
        if (level < m_staticGridLod.numLevels() - 1 && newRadarView.cellTooBig(cellCenter, cellSize))
        {
            auto[xi, yi] = cellIndex;
            cellIndices.push({level + 1, {xi * 2, yi * 2}});
            cellIndices.push({level + 1, {xi * 2 + 1, yi * 2}});
            cellIndices.push({level + 1, {xi * 2, yi * 2 + 1}});
            cellIndices.push({level + 1, {xi * 2 + 1, yi * 2 + 1}});
            continue;
        }
        Eigen::Vector2f halfCellSize = {cellSize * 0.5f, cellSize * 0.5f};
        m_reprojectContext.visitedCells.push_back({cellCenter - halfCellSize, cellCenter + halfCellSize});
        // Find the best log frame to reproject static points for this cell.
        auto[bestRadarId, bestScanIndex] = findBestStaticScanForTarget(newRadarView, cellCenter, bfsSeeds);
        const auto& bestGrid = m_initializeContext.at(bestRadarId)[bestScanIndex].staticPointcloud.gridLod.grid(level);
        if (bestGrid.find(cellIndex) != bestGrid.end())
        {
            // Transform static points to new sensor frame detections.
            for (const auto& point : bestGrid.at(cellIndex))
            {
                auto positionInSensor          = m_reprojectContext.worldToSensor * point.positionM;
                auto sphericalCoordinate       = cartesianToSpherical(positionInSensor);
                auto radialDirectionInWorld    = safeNormalize(point.positionM - m_reprojectContext.sensorToWorld.translation(), m_reprojectContext.sensorToWorld.linear().col(0));
                float32_t radialSpeedWrtSensor = (point.velocityMps - m_reprojectContext.input.egoState.velocityMps).dot(radialDirectionInWorld);
                RadarDetection detection       = {
                    sphericalCoordinate[0],
                    sphericalCoordinate[1],
                    sphericalCoordinate[2],
                    radialSpeedWrtSensor,
                    point.rcsDbm,
                    point.reference,
                    point.velocityMps};
                if (isInDetectionRange(radarIntrinsics, detection))
                {
                    m_reprojectContext.unfilteredScan.detections.push_back(detection);
                }
            }
        }
    }
}

std::vector<ViewIndex> RadarReprojector::genCommonBfsSeeds(const RadarView& newRadarView)
{
    std::vector<ViewIndex> seeds;
    seeds.reserve(m_initializeContext.size());
    for (const auto & [ radarId, frameContexts ] : m_initializeContext)
    {
        size_t closestScanIndex = std::min_element(
                                      frameContexts.begin(),
                                      frameContexts.end(),
                                      [&](const auto& f1, const auto& f2) {
                                          return f1.staticPointcloud.view.diff(newRadarView) < f2.staticPointcloud.view.diff(newRadarView);
                                      }) -
                                  frameContexts.begin();
        seeds.push_back({radarId, closestScanIndex});
    }
    if (seeds.size() > m_param.maxSearchSize)
    {
        std::sort(seeds.begin(), seeds.end(), [&](const auto& s1, const auto& s2) {
            const auto & [ radarId1, scanIndex1 ] = s1;
            const auto& view1 = m_initializeContext.at(radarId1)[scanIndex1].staticPointcloud.view;
            const auto & [ radarId2, scanIndex2 ] = s2;
            const auto& view2 = m_initializeContext.at(radarId2)[scanIndex2].staticPointcloud.view;
            return view1.diff(newRadarView) < view2.diff(newRadarView);
        });
        seeds.resize(m_param.maxSearchSize);
    }
    return seeds;
}

// NOTE Actors are small relative to static scene. Using single frame reprojector is enough.
void RadarReprojector::reprojectActorDetections(const std::string& actorId)
{
    const auto& radarIntrinsics = m_param.useLogIntrinsics ? m_logRadarIntrinsics.at(m_reprojectContext.input.radarParam.id) : m_reprojectContext.input.radarParam.intrinsics;
    const auto& actorState      = m_reprojectContext.input.actorToState.at(actorId);
    auto actorToSensor          = m_reprojectContext.worldToSensor * actorState.toWorld;
    auto actorBboxHalfSize      = actorState.bboxHalfSizeM.cwiseProduct(m_param.bboxScaleFactor) + m_param.bboxMargin;
    // Find the best log frame to reproject actor points for this cell.
    auto newRadarView = RadarView::create(actorToSensor.inverse(), radarIntrinsics);
    auto[bestRadarId, bestScanIndex] = findBestActorScan(newRadarView, actorId);
    if (bestRadarId == "")
    {
        return;
    }
    const auto& bestPointcloud = m_initializeContext.at(bestRadarId)[bestScanIndex].actorToPointcloud.at(actorId);
    // Transform actor points to new sensor frame detections.
    for (const auto& point : bestPointcloud.points)
    {
        if (point.positionM - actorState.bboxCenterM < actorBboxHalfSize)
        {
            auto positionInSensor          = actorToSensor * point.positionM;
            auto positionInWorld           = m_reprojectContext.sensorToWorld * positionInSensor;
            auto sphericalCoordinate       = cartesianToSpherical(positionInSensor);
            auto velocityInWorld           = actorState.toWorld.linear() * point.velocityMps + actorState.velocityMps;
            auto radialDirectionInWorld    = safeNormalize(positionInWorld - m_reprojectContext.sensorToWorld.translation(), m_reprojectContext.sensorToWorld.linear().col(0));
            float32_t radialSpeedWrtSensor = (velocityInWorld - m_reprojectContext.input.egoState.velocityMps).dot(radialDirectionInWorld);
            RadarDetection detection       = {
                sphericalCoordinate[0],
                sphericalCoordinate[1],
                sphericalCoordinate[2],
                radialSpeedWrtSensor,
                point.rcsDbm,
                point.reference,
                velocityInWorld};
            if (isInDetectionRange(radarIntrinsics, detection))
            {
                m_reprojectContext.unfilteredScan.detections.push_back(detection);
            }
        }
    }
}

ViewIndex RadarReprojector::findBestStaticScanForTarget(const RadarView& newRadarView, const Eigen::Vector2f& target, const std::vector<ViewIndex>& seeds) const
{
    using QueueItem = std::tuple<float, ViewIndex>;
    std::priority_queue<QueueItem> frontier;
    std::set<ViewIndex> visited;
    // Initialize frontier with seeds.
    for (const auto& seed : seeds)
    {
        auto[radarId, scanIndex] = seed;
        const auto& view     = m_initializeContext.at(radarId)[scanIndex].staticPointcloud.view;
        float32_t similarity = view.similarity(newRadarView, target);
        frontier.push({similarity, seed});
        visited.insert(seed);
    }
    // Current best similarity.
    auto[bestSimilarity, bestViewIndex] = frontier.top();
    // BFS.
    while (!frontier.empty())
    {
        if (frontier.size() > m_param.maxSearchSize)
        {
            break;
        }
        auto[similarity, viewIndex] = frontier.top();
        frontier.pop();
        if (similarity > bestSimilarity)
        {
            bestSimilarity = similarity;
            bestViewIndex  = viewIndex;
        }
        if (similarity < bestSimilarity * kBfsSimilarityRatio)
        {
            break;
        }
        const auto & [ radarId, scanIndex ] = viewIndex;
        if (scanIndex > 0)
        {
            const auto& neighborView     = m_initializeContext.at(radarId)[scanIndex - 1].staticPointcloud.view;
            float32_t neighborSimilarity = neighborView.similarity(newRadarView, target);
            ViewIndex neighborIndex      = {radarId, scanIndex - 1};
            if (visited.find(neighborIndex) == visited.end())
            {
                frontier.push({neighborSimilarity, neighborIndex});
                visited.insert(neighborIndex);
            }
        }
        if (scanIndex < m_initializeContext.at(radarId).size() - 1)
        {
            const auto& neighborView     = m_initializeContext.at(radarId)[scanIndex + 1].staticPointcloud.view;
            float32_t neighborSimilarity = neighborView.similarity(newRadarView, target);
            ViewIndex neighborIndex      = {radarId, scanIndex + 1};
            if (visited.find(neighborIndex) == visited.end())
            {
                frontier.push({neighborSimilarity, neighborIndex});
                visited.insert(neighborIndex);
            }
        }
    }
    return bestViewIndex;
}

ViewIndex RadarReprojector::findBestActorScan(const RadarView& newRadarView, const std::string& actorId) const
{
    std::string bestRadarId  = "";
    size_t bestScanIndex     = 0;
    float32_t bestSimilarity = 0.0f;
    for (const auto & [ radarId, frameContexts ] : m_initializeContext)
    {
        for (size_t scanIndex = 0; scanIndex < frameContexts.size(); ++scanIndex)
        {
            if (frameContexts[scanIndex].actorToState.find(actorId) == frameContexts[scanIndex].actorToState.end())
            {
                continue;
            }
            const auto& actorState = frameContexts[scanIndex].actorToState.at(actorId);
            const auto& actorView  = frameContexts[scanIndex].actorToPointcloud.at(actorId).view;
            float32_t similarity   = actorView.similarity(newRadarView, actorState.bboxCenterM.head<2>());
            if (similarity > bestSimilarity)
            {
                bestSimilarity = similarity;
                bestRadarId    = radarId;
                bestScanIndex  = scanIndex;
            }
        }
    }
    return {bestRadarId, bestScanIndex};
}

void RadarReprojector::computeIntrinsicsFromLog()
{
    m_logRadarIntrinsics.clear();
    for (const auto & [ radarId, logRecording ] : m_logDataPtr->logRadarData)
    {
        auto intrinsics = logRecording.param.intrinsics;
        for (const auto& scan : logRecording.scans)
        {
            for (const auto& detection : scan.detections)
            {
                intrinsics.minRangeM   = std::min(intrinsics.minRangeM, detection.distanceM);
                intrinsics.maxRangeM   = std::max(intrinsics.maxRangeM, detection.distanceM);
                intrinsics.halfHfovRad = std::max(intrinsics.halfHfovRad, std::abs(detection.azimuthRad));
                intrinsics.halfVfovRad = std::max(intrinsics.halfVfovRad, std::abs(detection.elevationRad));
            }
        }
        m_logRadarIntrinsics[radarId] = intrinsics;
    }
}

void RadarReprojector::downsampleDetections()
{
    const auto& radarParam = m_reprojectContext.input.radarParam;
    // Use a azimuth-elevation-range grid to ensure no 2 detections have all similar properties.
    // NOTE The v2 static reprojection usually already ensures this.
    std::map<std::tuple<int32_t, int32_t, int32_t>, std::vector<size_t>> grid; // key: indices of azimuth, elevation, range
    for (size_t detectionIndex = 0; detectionIndex < m_reprojectContext.unfilteredScan.detections.size(); ++detectionIndex)
    {
        const auto& detection  = m_reprojectContext.unfilteredScan.detections[detectionIndex];
        int32_t azimuthIndex   = static_cast<int32_t>(detection.azimuthRad / radarParam.intrinsics.hfovResRad);
        int32_t elevationIndex = static_cast<int32_t>(detection.elevationRad / radarParam.intrinsics.vfovResRad);
        int32_t rangeIndex     = static_cast<int32_t>(detection.distanceM / radarParam.intrinsics.rangeResM);
        auto gridIndex         = std::make_tuple(azimuthIndex, elevationIndex, rangeIndex);
        grid[gridIndex].push_back(detectionIndex);
    }
    for (const auto & [ _, cell ] : grid)
    {
        // For detections in the same cell (similar properties), only keep 1.
        // TODO(keq) Probabilistic selection based on rcs. Currently just keep the detection with highest rcs.
        auto strongestIndex = *std::max_element(
            cell.begin(),
            cell.end(),
            [&](const auto& i1, const auto& i2) {
                return m_reprojectContext.unfilteredScan.detections[i1].rcsDbm < m_reprojectContext.unfilteredScan.detections[i2].rcsDbm;
            });
        m_reprojectContext.filteredScan.detections.push_back(m_reprojectContext.unfilteredScan.detections[strongestIndex]);
    }
}

} // namespace algorithm
} // namespace radar_reprojection
