# Data Schema

## Scope

This document describes the PhysicalAI-AV fields consumed by `PhysicalAIAVAdapter` and their mapping to the gRPC input. It is not a complete schema for the upstream [PhysicalAI-Autonomous-Vehicles](https://huggingface.co/datasets/nvidia/PhysicalAI-Autonomous-Vehicles) dataset. `proto/radar_reprojector.proto` remains authoritative for the wire format.

## Input layout

For a clip `<clip>` and radar `<radar_id>`, the adapter reads:

| Path | Columns used |
| --- | --- |
| `labels/egomotion/<clip>.egomotion.parquet` | `timestamp`, `qx`, `qy`, `qz`, `qw`, `x`, `y`, `z`, `vx`, `vy`, `vz` |
| `labels/obstacle.offline/<clip>.obstacle.offline.parquet` | `timestamp_us`, `track_id`, `center_x`, `center_y`, `center_z`, `size_x`, `size_y`, `size_z`, `orientation_x`, `orientation_y`, `orientation_z`, `orientation_w` |
| `radar/<radar_id>/<clip>.<radar_id>.parquet` | `scan_index`, `timestamp`, `detection_index`, `azimuth`, `elevation`, `distance`, `radial_velocity`, `rcs` |
| `calibration/sensor_extrinsics/*.parquet` | `clip_id`, `sensor_name`, `qx`, `qy`, `qz`, `qw`, `x`, `y`, `z` |
| `calibration/vehicle_dimensions/*.parquet` | `clip_id`, `length`, `width`, `height` |

Calibration files are chunk-level files, and a dataset root holding clips from several chunks holds one file per chunk. The adapter searches every `*.parquet` in the directory, regardless of sort order, and selects rows whose `clip_id` matches the requested clip. If no file has a matching row it raises an error naming the clip and the files searched.

## Coordinate and unit conventions

- Timestamps are interpreted as microseconds and mapped to `timestamp_us`.
- Quaternions map directly to wire order `(x, y, z, w)`.
- Ego pose and velocity are mapped to world-frame `ActorState` fields.
- Offline obstacle poses are expressed in ego/rig frame. The adapter
  interpolates the ego pose at each obstacle timestamp and transforms the actor
  pose into world frame.
- Actor velocities are derived by finite differences after sorting each
  trajectory by timestamp.
- Radar detections remain in the radar's spherical frame: `azimuth`/`elevation` are radians, `distance` is metres, radial velocity is metres per second, and RCS maps to `rcs_dbm`.

Obstacle rows outside the ego-trajectory time window are skipped. Only `obstacle.offline` is consumed.

## Mapping to `LogData`

- Each radar directory produces one `LogRecording`, keyed by its radar ID.
- Detection rows are grouped by `scan_index` into `RadarScan` messages.
- `scan_index` and `detection_index` are preserved in `ReferenceIndex`; `actor_id` is initially empty.
- Egomotion rows produce `log_actor_data.ego_trajectory`.
- Obstacle rows are grouped by `track_id` into `log_actor_data.actor_to_trajectory`.
- Vehicle dimensions become half-size values on the ego bounding box.
- Sensor extrinsics whose `sensor_name` matches the radar ID become `RadarParam.to_ego`; if no row matches, the adapter uses an identity transform.

Initialization requires at least two states in the ego trajectory and in every included actor trajectory so poses can be interpolated.

## Radar intrinsics

`config/radar_intrinsics.json` supplies `minRangeM`, `maxRangeM`, `halfHfovRad`, `halfVfovRad`, `hfovResRad`, `vfovResRad`, and `rangeResM`.

The adapter uses the longest configuration-key suffix matching the radar ID (for example, `radar_front_center_srr_0` selects `srr_0`). If no key matches, it falls back to `srr_0`.

`RadarParam.max_num_returns` is the maximum detection count among the radar's loaded scans.

## Compatibility

The sample fixture revision and directory layout are recorded in `sample_data/README.md`. Upstream schemas may change between dataset revisions; the adapter's column accesses are the definitive list of fields required by this repository.

