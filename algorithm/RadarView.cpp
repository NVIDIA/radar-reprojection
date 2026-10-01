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

#include "RadarView.hpp"

#include <cmath>

#include "algorithm/Logger.hpp"

#include "TransformUtilities.hpp"

namespace radar_reprojection
{
namespace algorithm
{

namespace
{

static constexpr char LOG_TAG[] = "ndas_sim_light::radarreprojection::RadarView";

constexpr float32_t kOriginDiffWeight    = 1.0f;
constexpr float32_t kAzimuthDiffWeight   = 10.0f;
constexpr float32_t kOriginGapReference  = 3.0f;
constexpr float32_t kAzimuthGapReference = 1.5f;
constexpr int kDistanceExponent          = 4;
constexpr float32_t kAspectGapReference  = 0.2f;
constexpr float32_t kCellSplitRatio      = 0.2f;
constexpr float32_t kEpsilon             = 1.0E-6f;
}

RadarView RadarView::create(const Eigen::Affine3f& sensorToWorld, const RadarIntrinsics& intrinsics)
{
    // NOTE Theoretically should also consider pitch and roll angles as cosine factors, but impact is negligible.
    RadarView view;
    view.originM     = sensorToWorld.translation().head<2>();
    view.azimuthRad  = std::atan2(sensorToWorld.rotation().col(0)[1], sensorToWorld.rotation().col(0)[0]);
    view.radiusM     = intrinsics.maxRangeM;
    view.halfHfovRad = intrinsics.halfHfovRad;
    return view;
}

Eigen::Vector2f RadarView::sample(float32_t localAzimuthRad, float32_t distanceM) const
{
    return originM + distanceM * Eigen::Vector2f(std::cos(localAzimuthRad + azimuthRad), std::sin(localAzimuthRad + azimuthRad));
}

bool RadarView::withinFov(float32_t localAzimuthRad) const
{
    return std::abs(angleDiff(localAzimuthRad, 0)) < halfHfovRad;
}

bool RadarView::contains(const Eigen::Vector2f& positionM) const
{
    Eigen::Vector2f sensorToTarget = positionM - originM;
    float32_t sensorToTargetRad    = std::atan2(sensorToTarget[1], sensorToTarget[0]);
    return sensorToTarget.norm() <= radiusM && withinFov(sensorToTargetRad - azimuthRad);
}

float32_t RadarView::iou(const RadarView& other) const
{
    // NOTE Only consider the intersection/union in the azimuth fov.
    float32_t azimuthDiff = std::abs(angleDiff(azimuthRad, other.azimuthRad));
    if (halfHfovRad + other.halfHfovRad <= azimuthDiff)
    {
        return 0.0f;
    }
    // NOTE Here we assume both views' fov < 180 degrees, so we can ignore warping around 360.
    float32_t innerBound1     = std::min(halfHfovRad, other.halfHfovRad - azimuthDiff);
    float32_t innerBound2     = std::min(other.halfHfovRad, halfHfovRad - azimuthDiff);
    float32_t intersectionRad = innerBound1 + innerBound2 + azimuthDiff;
    float32_t outerBound1     = std::max(halfHfovRad, other.halfHfovRad - azimuthDiff);
    float32_t outerBound2     = std::max(other.halfHfovRad, halfHfovRad - azimuthDiff);
    float32_t unionRad        = outerBound1 + outerBound2 + azimuthDiff;
    return intersectionRad / unionRad;
}

std::vector<CellIndexLod> RadarView::cellIndices(float32_t cellSizeM) const
{
    std::vector<CellIndexLod> indices;
    if (cellSizeM <= 0.0f)
    {
        DW_LOGE << "cellSizeM must be positive." << dw_log::endl;
        return indices;
    }
    float32_t areaM2       = radiusM * radiusM * halfHfovRad;
    int estimatedCellCount = static_cast<int>(std::ceil(areaM2 / cellSizeM / cellSizeM));
    indices.reserve(estimatedCellCount);
    auto[minAABB, maxAABB] = AABB();
    int minXi = static_cast<int>(std::floor(minAABB[0] / cellSizeM)) - 1;
    int maxXi = static_cast<int>(std::ceil(maxAABB[0] / cellSizeM)) + 1;
    int minYi = static_cast<int>(std::floor(minAABB[1] / cellSizeM)) - 1;
    int maxYi = static_cast<int>(std::ceil(maxAABB[1] / cellSizeM)) + 1;
    for (int xi = minXi; xi <= maxXi; ++xi)
    {
        for (int yi = minYi; yi <= maxYi; ++yi)
        {
            indices.push_back({0, {xi, yi}});
        }
    }
    return indices;
}

std::tuple<Eigen::Vector2f, Eigen::Vector2f> RadarView::AABB() const
{
    static constexpr int kMaxExtremePoints     = 7; // origin, arc end points * 2, axis-aligned arc points * 4
    static constexpr int kExtremePointDim      = 2;
    static constexpr int kMaxAxisAlignedPoints = 4;
    Eigen::MatrixXf extremePoints(kMaxExtremePoints, kExtremePointDim);
    size_t rowIndex = 0;
    // Add origin and 2 extreme points on the arc.
    extremePoints.row(rowIndex++) = originM;
    extremePoints.row(rowIndex++) = sample(-halfHfovRad, radiusM);
    extremePoints.row(rowIndex++) = sample(halfHfovRad, radiusM);
    // Add 4 axis-aligned extreme points if within view fov.
    for (int i = 0; i < kMaxAxisAlignedPoints; ++i)
    {
        float32_t axisAzimuthRad = i * M_PI_2;
        if (withinFov(axisAzimuthRad - azimuthRad))
        {
            extremePoints.row(rowIndex++) = sample(axisAzimuthRad - azimuthRad, radiusM);
        }
    }
    auto validExtremePoints = extremePoints.topRows(rowIndex);
    Eigen::Vector2f minAABB;
    Eigen::Vector2f maxAABB;
    minAABB[0] = validExtremePoints.col(0).minCoeff();
    minAABB[1] = validExtremePoints.col(1).minCoeff();
    maxAABB[0] = validExtremePoints.col(0).maxCoeff();
    maxAABB[1] = validExtremePoints.col(1).maxCoeff();
    return {minAABB, maxAABB};
}

float32_t RadarView::angleDecay(const Eigen::Vector2f& target) const
{
    Eigen::Vector2f sensorToTarget = target - originM;
    float32_t sensorToTargetRad    = std::atan2(sensorToTarget[1], sensorToTarget[0]);
    float32_t localAzimuthRad      = std::min(std::abs(angleDiff(sensorToTargetRad, azimuthRad)), halfHfovRad);
    float32_t relativeAzimuth      = localAzimuthRad / halfHfovRad * M_PI;
    return relativeAzimuth < kEpsilon ? 1.0f : std::sin(relativeAzimuth) / relativeAzimuth;
}

float32_t RadarView::distance(const Eigen::Vector2f& target) const
{
    Eigen::Vector2f sensorToTarget = target - originM;
    return sensorToTarget.norm();
}

float32_t RadarView::aspect(const Eigen::Vector2f& target) const
{
    Eigen::Vector2f targetToOrigin = originM - target;
    float32_t targetToOriginRad    = std::atan2(targetToOrigin[1], targetToOrigin[0]);
    return targetToOriginRad;
}

float32_t RadarView::diff(const RadarView& other) const
{
    float32_t originDiff  = (originM - other.originM).norm();
    float32_t azimuthDiff = std::abs(angleDiff(azimuthRad, other.azimuthRad));
    return originDiff * kOriginDiffWeight + azimuthDiff * kAzimuthDiffWeight;
}

float32_t RadarView::similarity(const RadarView& other, const Eigen::Vector2f& target) const
{
    float32_t viewOriginGap        = (originM - other.originM).norm();
    float32_t viewAzimuthGap       = std::abs(angleDiff(azimuthRad, other.azimuthRad));
    float32_t aspectGap            = std::abs(angleDiff(aspect(target), other.aspect(target)));
    float32_t distanceSimilarity   = std::pow(relativeSimilarity(distance(target), other.distance(target)), kDistanceExponent);
    float32_t angleDecaySimilarity = relativeSimilarity(angleDecay(target), other.angleDecay(target));
    return std::exp(-viewOriginGap / kOriginGapReference - viewAzimuthGap / kAzimuthGapReference - aspectGap / kAspectGapReference) * distanceSimilarity * angleDecaySimilarity;
}

bool RadarView::intersectWithCell(const CellIndex& cellIndex, float32_t cellSizeM) const
{
    auto[xi, yi] = cellIndex;
    float32_t cellMinX    = xi * cellSizeM;
    float32_t cellMaxX    = (xi + 1) * cellSizeM;
    float32_t cellMinY    = yi * cellSizeM;
    float32_t cellMaxY    = (yi + 1) * cellSizeM;
    float32_t cellCenterX = (cellMinX + cellMaxX) / 2;
    float32_t cellCenterY = (cellMinY + cellMaxY) / 2;
    // Fan shape contains cell center.
    if (contains(Eigen::Vector2f(cellCenterX, cellCenterY)))
    {
        return true;
    }
    // Fan shape contains any of the 4 corners or center of the cell.
    if (contains(Eigen::Vector2f(cellMinX, cellMaxY)) || contains(Eigen::Vector2f(cellMaxX, cellMinY)) || contains(Eigen::Vector2f(cellMinX, cellMinY)) || contains(Eigen::Vector2f(cellMaxX, cellMaxY)))
    {
        return true;
    }
    // Cell contains the view origin.
    if (cellMinX <= originM[0] && cellMaxX >= originM[0] && cellMinY <= originM[1] && cellMaxY >= originM[1])
    {
        return true;
    }
    // NOTE In theory should also check edge intersections. But in our case fan's FOV is always >> 45 deg, so no corner cases anymore.
    return false;
}

bool RadarView::cellTooBig(const Eigen::Vector2f& target, float32_t cellSizeM) const
{
    return cellSizeM > distance(target) * kCellSplitRatio;
}

} // namespace algorithm
} // namespace radar_reprojection
