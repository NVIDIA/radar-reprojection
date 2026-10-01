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

"""Unit tests for PhysicalAIAVAdapter calibration lookup across chunk files.

Self-contained: writes synthetic calibration chunk parquets into a temp
dataset root, so it does not need sample_data/.
"""

import pathlib
import tempfile
import unittest

import pyarrow as pa
import pyarrow.parquet as pq

from examples.python import physical_ai_av_adapter as paa


CLIP_A = "aaaaaaaa-0000-0000-0000-000000000000"   # in chunk_0000 (sorts first)
CLIP_B = "bbbbbbbb-0000-0000-0000-000000000000"   # in chunk_2073 (sorts later)
CLIP_MISSING = "cccccccc-0000-0000-0000-000000000000"


def _write_extrinsics(path: pathlib.Path, clip_id: str, sensor: str, x: float) -> None:
    pq.write_table(pa.table({
        "clip_id": [clip_id], "sensor_name": [sensor],
        "qx": [0.0], "qy": [0.0], "qz": [0.0], "qw": [1.0],
        "x": [x], "y": [0.0], "z": [0.5],
    }), str(path))


def _write_dims(path: pathlib.Path, clip_id: str, length: float) -> None:
    pq.write_table(pa.table({
        "clip_id": [clip_id], "length": [length], "width": [2.0], "height": [1.5],
    }), str(path))


def _make_root(tmp: str) -> pathlib.Path:
    root = pathlib.Path(tmp)
    ext = root / "calibration" / "sensor_extrinsics"
    dims = root / "calibration" / "vehicle_dimensions"
    ext.mkdir(parents=True)
    dims.mkdir(parents=True)
    _write_extrinsics(ext / "sensor_extrinsics.chunk_0000.parquet", CLIP_A, "radar_a", 1.0)
    _write_extrinsics(ext / "sensor_extrinsics.chunk_2073.parquet", CLIP_B, "radar_b", 2.0)
    _write_dims(dims / "vehicle_dimensions.chunk_0000.parquet", CLIP_A, 4.0)
    _write_dims(dims / "vehicle_dimensions.chunk_2073.parquet", CLIP_B, 5.0)
    return root


def _make_adapter(root: pathlib.Path, clip: str) -> paa.PhysicalAIAVAdapter:
    return paa.PhysicalAIAVAdapter(
        dataset_root=root, clip_uuid=clip,
        intrinsics_config=root / "unused_intrinsics.json")


class CalibrationChunkLookupTest(unittest.TestCase):
    def test_clip_in_first_chunk(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = _make_adapter(_make_root(tmp), CLIP_A)
            self.assertEqual(list(a._load_extrinsics()), ["radar_a"])
            self.assertAlmostEqual(a._load_extrinsics()["radar_a"].translation.x, 1.0)
            self.assertAlmostEqual(a._load_vehicle_dims()["length"], 4.0)

    def test_clip_in_later_chunk(self):
        with tempfile.TemporaryDirectory() as tmp:
            b = _make_adapter(_make_root(tmp), CLIP_B)
            self.assertEqual(list(b._load_extrinsics()), ["radar_b"])
            self.assertAlmostEqual(b._load_extrinsics()["radar_b"].translation.x, 2.0)
            self.assertAlmostEqual(b._load_vehicle_dims()["length"], 5.0)

    def test_missing_clip_names_clip_and_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = _make_adapter(_make_root(tmp), CLIP_MISSING)
            for loader in (c._load_extrinsics, c._load_vehicle_dims):
                with self.assertRaises(RuntimeError) as ctx:
                    loader()
                msg = str(ctx.exception)
                self.assertIn(CLIP_MISSING, msg)
                self.assertIn("chunk_0000.parquet", msg)
                self.assertIn("chunk_2073.parquet", msg)

    def test_missing_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            a = _make_adapter(pathlib.Path(tmp), CLIP_A)
            with self.assertRaises(FileNotFoundError):
                a._load_extrinsics()


if __name__ == "__main__":
    unittest.main()
