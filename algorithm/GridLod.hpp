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

#ifndef RADAR_REPROJECTION_ALGORITHM_GRID_LOD_HPP_
#define RADAR_REPROJECTION_ALGORITHM_GRID_LOD_HPP_

#include <map>
#include <vector>

#include <Eigen/Dense>

#include "algorithm/BasicTypes.hpp"

namespace radar_reprojection
{
namespace algorithm
{

using CellIndex    = std::tuple<int32_t, int32_t>;
using CellIndexLod = std::tuple<size_t, CellIndex>;

/**
 * @class GridLod
 * @brief a grid hierarchy data structure for adaptive spatial partitioning
 * @tparam T The type of the data stored in each grid cell.
 *
 * This class is an extended version of a 2D grid that contains multiple layers, where each layer is 4x denser than the previous one.
 */
template <typename T>
class GridLod
{
public:
    GridLod() = default;

    /**
     * @brief Constructor that initializes the GridLod with a maximum and minimum cell size.
     *
     * This constructor creates a GridLod structure with a range of cell sizes, starting from cellSizeMax
     * and decreasing by a factor of 2 until reaching cellSizeMin.
     *
     * @param cellSizeMax The initial cell size for the highest level.
     * @param cellSizeMin The threshold cell size for the lowest level (not necessarily the final cell size).
     */
    GridLod(float32_t cellSizeMax, float32_t cellSizeMin);

    /**
     * @brief Returns the number of levels in the grid hierarchy.
     * @return Number of levels in the grid hierarchy.
     */
    size_t numLevels() const;

    /**
     * @brief Returns the cell size for a given level.
     * @param level The level of the grid to query.
     * @return The cell size for the specified level.
     */
    float32_t cellSize(size_t level) const;

    /**
     * @brief Clears the GridLod structure and resets it to the initial state.
     * @param cellSizeMax The initial cell size for the highest level.
     * @param cellSizeMin The threshold cell size for the lowest level (not necessarily the final cell size).
     */
    void clear(float32_t cellSizeMax, float32_t cellSizeMin);

    /**
     * @brief Returns a reference/const reference to the item at a specific level and cell index.
     * @param level The level of the grid to query.
     * @param cellIndex The index of the cell to retrieve.
     * @return A reference/const reference to the item at the specified level and cell index.
     */
    const T& item(size_t level, CellIndex cellIndex) const;
    T& item(size_t level, CellIndex cellIndex);

    /**
     * @brief Returns a reference/const reference to the grid at a specific level.
     * @param level The level of the grid to query.
     * @return A reference/const reference to the grid at the specified level.
     */
    const std::map<CellIndex, T>& grid(size_t level) const;
    std::map<CellIndex, T>& grid(size_t level);

private:
    std::vector<float> m_cellSizes;
    std::vector<std::map<CellIndex, T>> m_grids;
};

template <typename T>
GridLod<T>::GridLod(float32_t cellSizeMax, float32_t cellSizeMin)
{
    clear(cellSizeMax, cellSizeMin);
}

template <typename T>
size_t GridLod<T>::numLevels() const
{
    return m_cellSizes.size();
}

template <typename T>
void GridLod<T>::clear(float32_t cellSizeMax, float32_t cellSizeMin)
{
    m_cellSizes.clear();
    m_grids.clear();
    float32_t cellSize = cellSizeMax;
    while (cellSize > cellSizeMin)
    {
        m_cellSizes.push_back(cellSize);
        m_grids.push_back(std::map<CellIndex, T>());
        cellSize /= 2;
    }
}

template <typename T>
float32_t GridLod<T>::cellSize(size_t level) const
{
    return m_cellSizes[level];
}

template <typename T>
const T& GridLod<T>::item(size_t level, CellIndex cellIndex) const
{
    return m_grids[level][cellIndex];
}

template <typename T>
T& GridLod<T>::item(size_t level, CellIndex cellIndex)
{
    return m_grids[level][cellIndex];
}

template <typename T>
const std::map<CellIndex, T>& GridLod<T>::grid(size_t level) const
{
    return m_grids[level];
}

template <typename T>
std::map<CellIndex, T>& GridLod<T>::grid(size_t level)
{
    return m_grids[level];
}

} // namespace algorithm
} // namespace radar_reprojection

#endif // RADAR_REPROJECTION_ALGORITHM_GRID_LOD_HPP_
