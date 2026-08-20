# Radar Reprojection Algorithm

## Overview

The algorithm adapts real radar detections to a simulated ego and actor state. It does not generate detections from a scene model: every output point retains a link to a logged detection, while its geometry and radial velocity are recomputed for the requested viewpoint.

`RadarReprojector` has two phases:

1. During initialization, logged scans are converted into reusable world-frame and actor-frame representations.
2. During reprojection, the algorithm selects the most compatible logged observations for the requested view and transforms them into a new radar scan.

The central distinction is spatial: static scenery is assembled region by region from multiple log frames, while each dynamic actor is reconstructed from one actor-relative observation.

## Preprocessing

For every radar scan, initialization interpolates the ego and actor trajectories to the scan timestamp and computes the logged sensor pose

$$
T_{\text{sensor}\rightarrow\text{world}} =
T_{\text{ego}\rightarrow\text{world}}\,T_{\text{sensor}\rightarrow\text{ego}}.
$$

Each spherical detection is transformed to world coordinates and tested against the configured actor bounding boxes. Actor-associated points are stored in that actor's local frame; all remaining points are static and remain in world coordinates. Bounding-box scale and margin parameters make this association tolerant to annotation and measurement noise.

Each static point is inserted into every level of an adaptive spatial grid (`GridLod`). The current hierarchy uses 20, 10, 5, and 2.5 metre cells. A second global grid stores occupancy across all scans, allowing reprojection to reject empty space without inspecting individual frames.

For velocity handling, the logged radial measurement is first compensated by ego motion and represented along the logged world-frame ray:

$$
\mathbf{v}_{p}^{\,w}=\hat{\mathbf r}^{\,w}\left(v_{\text{radial}}+\mathbf{v}_{\text{ego}}^{\,w}\cdot\hat{\mathbf r}^{\,w}\right).
$$

When log-derived intrinsics are requested, the configured range and field-of-view limits are expanded to include the extrema observed in the log.

## Adaptive multi-frame static reprojection

The requested radar is represented by a `RadarView`: a 2D fan defined by world-frame origin, horizontal boresight, maximum range, and half horizontal field of view.

Static reprojection walks the fan through the global grid from coarse to fine:

1. Cover the visible fan with coarse 20 metre cells.
2. Reject cells that are globally empty or do not intersect the fan.
3. Subdivide an occupied cell into four children when it is too large for its distance from the radar. Nearby space is therefore represented more finely, while distant space remains coarse.
4. At each resulting leaf cell, independently select the logged scan whose view best matches the requested view at that cell's center.
5. Take the logged detections from that scan and region, then transform them into the requested sensor frame.

This per-cell decision is the key multi-frame behavior: one output scan can be a spatial mosaic in which nearby and distant regions, or different parts of the field of view, come from different logged timestamps and radars.

### Source-view search

The search first chooses one coarse starting frame per logged radar by minimizing origin and boresight difference. If their number exceeds the configured search budget (`maxSearchSize`), only the closest starting frames are retained.

For each leaf cell, a best-first search scores the starting frames at the cell center, then explores the preceding and following scans in each radar recording. Exploration stops when the frontier exceeds the search budget or the next candidate falls below half of the best similarity found. A search budget of one approximates the original single-candidate behavior.

## View similarity

View similarity compares a logged view $A$ and requested view $B$ relative to target point $\mathbf t$:

$$
S(A,B,\mathbf t)=
\exp\left(-\frac{\Delta o}{3}-\frac{\Delta\psi}{1.5}-\frac{\Delta\alpha}{0.2}\right)
R(d_A,d_B)^4
R(q_A,q_B),
$$

where $\Delta o$ is sensor-origin separation, $\Delta\psi$ is boresight difference, $\Delta\alpha$ is the difference between target-relative viewing aspects, and

$$
R(x,y)=1-\frac{|x-y|}{x+y}.
$$

$d$ is sensor-to-target distance. $q$ is a sinc-shaped off-axis response, $\sin\theta/\theta$, computed from normalized horizontal offset within the radar fan. The score therefore favors observations taken from a similar location, orientation, distance, target aspect, and part of the antenna field of view—not merely the nearest timestamp.

## Actor reprojection

Dynamic points follow a separate actor-relative path. For each simulated actor, the algorithm constructs the requested radar view in the actor's local frame and searches all logged frames containing that actor. The frame with maximum view similarity at the actor's bounding-box center supplies the actor-local point pattern.

Those points are transformed through the simulated actor pose into the requested sensor frame. This preserves the logged observation pattern while allowing both the ego and actor to move away from their recorded trajectories. A single source frame is used per actor because an actor occupies a compact region compared with the static scene.

## Measurement reconstruction

After a source point is selected, its Cartesian position is transformed into the requested sensor frame and converted back to azimuth, elevation, and range. Radial velocity is recomputed from relative world velocity:

$$
v_{\text{radial,new}}=
\left(\mathbf v_{p}^{\,w}-\mathbf v_{\text{ego,new}}^{\,w}\right)\cdot
\hat{\mathbf r}_{\text{new}}^{\,w}.
$$

This computation best preserves the accuracy of the static detections' radial velocity.

RCS is preserved from the logged detection. Points outside the requested radar's range or angular limits are discarded. Output detections are sorted by range.

## Provenance and optional downsampling

Every point carries provenance (`ReferenceIndex`) containing its source radar, scan, detection index, and optional actor ID. The reference survives reprojection, making a multi-frame output traceable point by point.

When downsampling is enabled, detections are quantized into azimuth/elevation/range bins using the radar resolutions. One detection—the highest-RCS return—is retained per occupied bin.

## Why adaptive multi-frame sampling

### Controlled trajectory deviation

Unlike direct timestamp-based replay, view-based sampling allows the simulated ego and actors to move away from their logged trajectories while continuing to use observations captured from a compatible perspective. Longitudinal ego changes such as moderate speed differences are generally supported while the simulated pose remains inside the spatial and temporal coverage of the log. A lateral change of roughly one lane (about 2 m) is a useful practical scale, not a guaranteed bound; scene layout and available viewpoints remain decisive. Actor deviations are similarly constrained by whether the log contains observations of that actor from a comparable relative angle.

### Log-grounded detections

The algorithm never fabricates a radar return. Position, range, angles, and radial velocity are transformed for the requested state, but each output retains the RCS, measurement characteristics, and provenance of a real logged detection. This preserves log-specific effects that would be difficult to reproduce with a purely geometric sensor model.

### Spatial precision where it matters

The adaptive grid assigns fine cells to nearby regions, where viewpoint changes have greater geometric impact, and coarse cells to distant regions. Independent source selection per region avoids committing the entire output to one logged frame: each part of the scene can use the observation captured from its most compatible view.

## Limitations

### Bounded by log coverage

Because the algorithm does not invent detections, it cannot reconstruct a region or actor that was not sufficiently observed in the log. Large ego or actor deviations can therefore produce sparse, mismatched, or missing returns. The practical deviation ranges above are guidance rather than validity guarantees.

### Occlusion is not recomputed

The algorithm has no scene geometry for ray casting or visibility testing. Occlusion present in a selected source observation is naturally retained, but visibility is not recomputed after ego or actor motion. A sufficiently changed configuration can therefore preserve a return that should become occluded, or omit one that should become visible.

### Radial velocity is underconstrained

A radar detection provides only line-of-sight velocity, so its full three-dimensional velocity cannot be recovered from one measurement. Initialization compensates for ego motion and stores the inferred velocity along the original radar ray; tangential velocity remains unobserved. Radial velocity after a viewpoint change is therefore an approximation, especially for dynamic points. Actor-level motion is incorporated, but it cannot restore the missing tangential component of the original detection.

### View selection is predominantly horizontal

`RadarView` models origin, yaw, range, and horizontal field of view. Pitch and roll do not participate in source-view scoring, although final detections are still filtered against the radar's vertical field of view.

## Key files

- `RadarReprojector.{hpp,cpp}` — preprocessing, source selection, reprojection, and downsampling.
- `RadarView.{hpp,cpp}` — fan geometry, adaptive-cell tests, and view similarity.
- `GridLod.hpp` — multi-resolution spatial hierarchy.
- `ReprojectorTypes.hpp` — intermediate point clouds and algorithm contexts.
- `TransformUtilities.{hpp,cpp}` — interpolation, coordinate conversion, and detection-range checks.
