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

#ifndef RADAR_REPROJECTION_ALGORITHM_RADAR_VIEW_HPP_
#define RADAR_REPROJECTION_ALGORITHM_RADAR_VIEW_HPP_

#include <tuple>
#include <vector>
#include <Eigen/Dense>

#include "algorithm/BasicTypes.hpp"

#include "GridLod.hpp"
#include "RadarTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

/**
 * @brief Simplified 2D radar fan-shape view.
 * TODO(keq) Rename to eliminate ambiguity (with "read-only radar data").
 */
struct RadarView
{
    /**
     * @brief Create a RadarView from radar pose and intrinsics.
     *
     * @param sensorToBase The sensor to base transform (based could be world frame or any actor's local frame).
     * @param intrinsics The radar intrinsics.
     * @return The created RadarView.
     */
    static RadarView create(const Eigen::Affine3f& sensorToBase, const RadarIntrinsics& intrinsics);

    /**
     * @brief Sample a point within the radar view, specified by angle and distance.
     *
     * @param localAzimuthRad The azimuth angle in relative to radar view's bore sight direction.
     * @param distanceM The distance from the origin.
     * @return The sampled point.
     */
    Eigen::Vector2f sample(float32_t localAzimuthRad, float32_t distanceM) const;

    /**
     * @brief Check if a given angle is within the radar view's fov.
     *
     * @param azimuthRad The azimuth angle relative to base frame.
     * @return True if the angle is within, false otherwise.
     */
    bool withinFov(float32_t azimuthRad) const;

    /**
     * @brief Check if a given position is within the radar view fan shape.
     *
     * @param positionM The position in base frame.
     * @return True if the position is within, false otherwise.
     */
    bool contains(const Eigen::Vector2f& positionM) const;

    /**
     * @brief Compute the Intersection Over Union (IoU) between two RadarViews.
     *
     * @param other The other RadarView to compare with.
     * @return The IoU value.
     */
    float32_t iou(const RadarView& other) const;

    /**
     * @brief Compute all cell indices within the radar view fan shape.
     *
     * @param cellSizeM The cell size in meters.
     * @return The vector of cell indices.
     */
    std::vector<CellIndexLod> cellIndices(float32_t cellSizeM) const;

    /**
     * @brief Compute the axis-aligned bounding box as a pair of min and max corner points.
     *
     * @return The pair of min and max corner points.
     */
    std::tuple<Eigen::Vector2f, Eigen::Vector2f> AABB() const;

    /**
     * @brief Compute the angle decay factor between the radar view and a target point.
     *
     * @param target The target point.
     * @return The angle decay factor.
     */
    float32_t angleDecay(const Eigen::Vector2f& target) const;

    /**
     * @brief Compute the distance between the radar view and a target point.
     *
     * @param target The target point.
     * @return The distance.
     */
    float32_t distance(const Eigen::Vector2f& target) const;

    /**
     * @brief Compute the aspect of a target point seen by the radar view.
     *
     * @param target The target point.
     * @return The aspect angle.
     */
    float32_t aspect(const Eigen::Vector2f& target) const;

    /**
     * @brief Compute the difference between 2 radar views, regarless of which target is viewed.
     *
     * @param other The other radar view.
     * @return The difference.
     */
    float32_t diff(const RadarView& other) const;

    /**
     * @brief Compute the similarity between 2 radar views w.r.t. a target point.
     *
     * @param other The other radar view.
     * @param target The target point.
     * @return The similarity.
     */
    float32_t similarity(const RadarView& other, const Eigen::Vector2f& target) const;

    /**
     * @brief Check if the radar view (fan shape) intersects with a cell (axis-aligned square).
     *
     * @param cellIndex The cell index.
     * @param cellSizeM The cell size in meters.
     * @return True if the radar view intersects with the cell, false otherwise.
     */
    bool intersectWithCell(const CellIndex& cellIndex, float32_t cellSizeM) const;

    /**
     * @brief Check if the cell size is too big at a target point and can be merged to speed up.
     *
     * @param target The target point.
     * @param cellSizeM The cell size in meters.
     * @return True if the cell size is too big, false otherwise.
     */
    bool cellTooBig(const Eigen::Vector2f& target, float32_t cellSizeM) const;

    Eigen::Vector2f originM;
    float32_t azimuthRad; // fan shape bore sight direction
    float32_t radiusM;
    float32_t halfHfovRad;
};

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_RADAR_VIEW_HPP_
