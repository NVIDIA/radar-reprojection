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

#ifndef RADAR_REPROJECTION_ALGORITHM_LOGGER_HPP_
#define RADAR_REPROJECTION_ALGORITHM_LOGGER_HPP_

#include <iostream>
#include <ostream>

namespace dw_log {

// Sentinel that maps to std::endl in << chains; matches legacy
// `dw_log::endl` usage at call sites.
inline std::ostream& endl(std::ostream& os) { return os << std::endl; }

} // namespace dw_log

// Drop-in replacements for the DW logger macros. Each emits to std::cerr.
// The legacy code uses these in chained << style ending in
// `dw_log::endl`; we provide `dw_log::endl` as the
// replacement sentinel and a sed sweep in Task F5 rewrites call sites.
#define DW_LOGE (std::cerr << "[E] ")
#define DW_LOGW (std::cerr << "[W] ")
#define DW_LOGI (std::cerr << "[I] ")
#define DW_LOGD (std::cerr << "[D] ")
#define DW_LOGV (std::cerr << "[V] ")

#endif // RADAR_REPROJECTION_ALGORITHM_LOGGER_HPP_
