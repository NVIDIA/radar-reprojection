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

#ifndef RADAR_REPROJECTION_ALGORITHM_TRANSFORM_UTILITIES_HPP_
#define RADAR_REPROJECTION_ALGORITHM_TRANSFORM_UTILITIES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "algorithm/BasicTypes.hpp"

#include "ActorTypes.hpp"
#include "RadarTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

/**
 * @brief Linear interpolation between two values.
 *
 * @param v1 The first value.
 * @param v2 The second value.
 * @param ratio The interpolation ratio.
 * @return The interpolated value.
 */
float32_t lerp(float32_t v1, float32_t v2, float32_t ratio);

/**
 * @brief Linear interpolation between two vectors (e.g. positions).
 *
 * @param pos1 The first vector.
 * @param pos2 The second vector.
 * @param ratio The interpolation ratio.
 * @return The interpolated vector.
 */
Eigen::Vector3f lerp(const Eigen::Vector3f& pos1, const Eigen::Vector3f& pos2, float32_t ratio);

/**
 * @brief Spherical interpolation between two vectors (e.g. velocities).
 *
 * @param vel1 The first vector.
 * @param vel2 The second vector.
 * @param ratio The interpolation ratio.
 * @return The interpolated vector.
 */
Eigen::Vector3f slerp(const Eigen::Vector3f& vel1, const Eigen::Vector3f& vel2, float32_t ratio);

/**
 * @brief Spherical interpolation between two ActorState instances.
 *
 * @param state1 The first ActorState.
 * @param state2 The second ActorState.
 * @param ratio The interpolation ratio.
 * @return The interpolated ActorState.
 */
ActorState slerp(const ActorState& state1, const ActorState& state2, float32_t ratio);

/**
 * @brief Spherical interpolation of an ActorState from a trajectory at a given timestamp.
 *
 * @param trajectory The trajectory.
 * @param timestampUs The timestamp.
 * @return The interpolated ActorState.
 */
ActorState slerp(const ActorTrajectory& trajectory, dwTime_t timestampUs);

/**
 * @brief Convert a spherical position to Cartesian coordinates.
 *
 * @param position The spherical position.
 * @return The Cartesian position.
 */
Eigen::Vector3f sphericalToCartesian(const Eigen::Vector3f& position);

/**
 * @brief Convert a RadarDetection's spherical position to Cartesian coordinates.
 *
 * @param detection The RadarDetection.
 * @return Its Cartesian position.
 */
Eigen::Vector3f sphericalToCartesian(const RadarDetection& detection);

/**
 * @brief Convert a Cartesian position to spherical coordinates.
 *
 * @param position The Cartesian position.
 * @return The spherical coordinates {azimuth, elevation, distance} as a vector.
 */
Eigen::Vector3f cartesianToSpherical(const Eigen::Vector3f& position);

/**
 * @brief Convert degrees to radians.
 *
 * @param deg The angle in degrees.
 * @return The angle in radians.
 */
float32_t degToRad(float32_t deg);

/**
 * @brief Convert radians to degrees.
 *
 * @param rad The angle in radians.
 * @return The angle in degrees.
 */
float32_t radToDeg(float32_t rad);

/**
 * @brief Convert Cartesian coordinates and Euler angles to an affine transformation.
 *
 * @param xyz The Cartesian position.
 * @param rpy The Euler angles {roll, pitch, yaw} in radians as a vector.
 * @return The affine transformation.
 */
Eigen::Affine3f xyzrpyToAffine(const Eigen::Vector3f& xyz, const Eigen::Vector3f& rpy);

/**
 * @brief Check if a RadarDetection is within the radar's detection range.
 *
 * @param intrinsics The radar intrinsics.
 * @param detection The RadarDetection.
 * @return True if the detection is within range, false otherwise.
 */
bool isInDetectionRange(const RadarIntrinsics& intrinsics, const RadarDetection& detection);

/**
 * @brief Check whether a vector is component-wise less than another.
 *
 * @param v1 The first vector.
 * @param v2 The second vector.
 * @return True if v1 is component-wise less than v2, false otherwise.
 */
bool operator<(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

/**
 * @brief Compute the component-wise absolute value of a vector.
 *
 * @param v The vector.
 * @return The component-wise absolute value of the vector.
 */
Eigen::Vector3f abs(const Eigen::Vector3f& v);

/**
 * @brief Normalize a vector, returning a default vector if the input is close to zero.
 *
 * @param v The vector.
 * @param defaultVec The default vector to return if the input is close to zero.
 * @return The normalized vector.
 */
Eigen::Vector3f safeNormalize(const Eigen::Vector3f& v, const Eigen::Vector3f& defaultVec = Eigen::Vector3f::Zero());

/**
 * @brief Compute the difference between two angles with proper wrap-around.
 *
 * @param theta1 The first angle.
 * @param theta2 The second angle.
 * @return The difference between the two angles.
 */
float32_t angleDiff(float32_t theta1, float32_t theta2);

/**
 * @brief Compute the relative similarity between two values. 1 if they are the same, 0 if one of them is 0.
 *
 * @param x The first value.
 * @param y The second value.
 * @return The relative similarity between the two values.
 */
float32_t relativeSimilarity(float32_t x, float32_t y);

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_TRANSFORM_UTILITIES_HPP_
