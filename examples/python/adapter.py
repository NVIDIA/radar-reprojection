# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Abstract data adapter for the radar-reprojection gRPC server.

Subclass RadarReprojectionDataAdapter to plug in a new dataset format. The
adapter's job is to convert a clip's logged radar + ego/actor data into the
proto messages the server consumes. Iteration over ego poses for Reproject
calls is handled by reproject_iter.iter_reproject_requests, not by adapters.
"""

from abc import ABC, abstractmethod

from proto import radar_reprojector_pb2 as pb


class RadarReprojectionDataAdapter(ABC):
    @abstractmethod
    def load_log_data(self) -> pb.LogData:
        """Return a LogData proto: per-radar recordings + ego/actor trajectories.

        Implementations should cache the result so repeat calls are cheap.
        """

    @abstractmethod
    def reprojector_param(self) -> pb.ReprojectorParam:
        """Return the algorithm-tuning ReprojectorParam.

        Subclasses that don't need to tune the algorithm may simply return
        default_reprojector_param().
        """


def default_reprojector_param() -> pb.ReprojectorParam:
    """ReprojectorParam with conservative defaults safe for any clip."""
    p = pb.ReprojectorParam()
    p.use_log_intrinsics = True
    p.do_down_sampling = False
    p.max_search_size = 30
    p.dump = False
    p.bbox_scale_factor.x = 1.0
    p.bbox_scale_factor.y = 1.0
    p.bbox_scale_factor.z = 1.0
    p.bbox_margin.x = 0.0
    p.bbox_margin.y = 0.0
    p.bbox_margin.z = 0.0
    return p
