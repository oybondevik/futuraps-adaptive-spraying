# futuraps_spray_coverage

ROS 2 package for estimating and visualizing spray dose coverage on a canopy
point cloud. By default, the package continuously merges a live obstacle cloud
into a voxelized coverage cloud, accumulates dose while spraying is enabled,
publishes a colored dose cloud, and publishes simple coverage fractions.

## Node

Executable:

```bash
spray_coverage_node
```

Launch:

```bash
ros2 launch futuraps_spray_coverage spray_coverage.launch.py
```

Optional config override:

```bash
ros2 launch futuraps_spray_coverage spray_coverage.launch.py \
  config_file:=/path/to/platform_ur10_6_nozzle.yaml
```

## Inputs

| Topic | Type | Purpose |
| --- | --- | --- |
| `/cloud` | `sensor_msgs/msg/PointCloud2` | Live canopy/obstacle point cloud. |
| `/spray/enabled` | `std_msgs/msg/Bool` | Enables dose accumulation when true. |
| TF `coverage_frame -> nozzle_frame` | TF transform | Defines the current boom/nozzle pose in live mode. |

In live mode, each incoming cloud is transformed into `coverage_frame` at the
cloud timestamp and merged into a voxel map. Existing voxel dose is preserved
when new observations update the voxel position and timestamp.

Use a locally continuous frame such as `odom` for `coverage_frame`. RTAB-Map's
`map` frame can jump during pose-graph optimization, which can shift old dose
history relative to new observations. `odom` is usually better for accumulating
coverage over short row segments.

Set `accumulate_live_cloud: false` to keep the old static behavior: the first
valid cloud on `cloud_topic` is stored as the canopy snapshot and later clouds
are ignored.

## Outputs

| Topic | Type | Purpose |
| --- | --- | --- |
| `/spray_coverage/dose_cloud` | `sensor_msgs/msg/PointCloud2` | Colored point cloud showing accumulated dose. |
| `/spray_coverage/coverage` | `geometry_msgs/msg/Vector3` | Fractions of under-, well-, and over-sprayed points. |
| `/spray_coverage/spray_marker` | `visualization_msgs/msg/Marker` | RViz line marker for the spray footprint. |

The coverage vector uses:

- `x`: undersprayed fraction
- `y`: well-sprayed fraction
- `z`: oversprayed fraction

## Spray Model

Each nozzle is described by:

- a local offset from `nozzle_frame`
- a local spray axis
- a local fan-wide direction

The node transforms these vectors into the active accumulation frame
(`coverage_frame` in live mode, `world_frame` in static mode). The fan-wide
direction is projected onto the plane perpendicular to the spray axis. The
narrow direction is computed as:

```text
narrow = spray_axis x fan_wide
```

For a canopy point `p` and nozzle origin `o`, define:

```text
r = p - o
s = r dot spray_axis
p_wide = r dot fan_wide
p_narrow = r dot narrow
```

Only points in front of the nozzle and within `max_range` are considered:

```text
0 < s <= max_range
```

The spray footprint is an elliptical paraboloid. At distance `s`, the semi-axis
lengths are:

```text
a(s) = s * tan(fan_angle_deg / 2)
b(s) = s * tan(narrow_angle_deg / 2)
```

where:

- `a(s)` is the wide fan semi-axis
- `b(s)` is the narrow semi-axis

The normalized elliptical radius is:

```text
rho_e^2 = p_wide^2 / a(s)^2 + p_narrow^2 / b(s)^2
```

Points outside the ellipse are ignored:

```text
rho_e^2 > 1
```

For points inside the ellipse, instantaneous deposition intensity is:

```text
I = peak_intensity * (1 - rho_e^2)
```

The centerline intensity is therefore:

```text
I_center = peak_intensity
```

With the default config, the center intensity is `1.0`.

Dose is accumulated over time as:

```text
dose += I * dt
```

## Occlusion Model

When occlusion is enabled, the node bins points by their narrow/wide footprint
coordinates and keeps the front-most point depth `s` in each bin. During dose
accumulation, points behind that front depth by more than
`occlusion_depth_tolerance` are skipped.

Occlusion uses only points inside the active spray ellipse. This keeps the
occlusion model aligned with the deposition footprint.

## Main Parameters

Configured in `config/default.yaml`.

| Parameter | Meaning |
| --- | --- |
| `world_frame` | Frame used for static-mode TF lookup and output cloud. |
| `coverage_frame` | Locally stable frame used for live cloud and dose accumulation. |
| `nozzle_frame` | Frame containing nozzle offsets and axes. |
| `accumulate_live_cloud` | Enables continuous live-cloud voxel merging. |
| `live_cloud_topic` | Live canopy/obstacle cloud topic. |
| `cloud_qos_reliability` | Cloud subscription reliability: `reliable`, `best_effort`, or `system_default`. |
| `cloud_qos_depth` | Cloud subscription queue depth. |
| `update_rate_hz` | Dose update loop rate. |
| `peak_intensity` | Centerline deposition intensity. |
| `fan_angle_deg` | Wide spray fan angle. |
| `narrow_angle_deg` | Narrow spray angle. |
| `max_range` | Maximum spray distance. |
| `voxel_leaf_size` | Cloud downsampling voxel size. |
| `max_point_age` | Maximum age for points used in live occlusion/front-surface checks. |
| `use_recent_cloud_for_occlusion` | Uses only recently observed live points for occlusion. |
| `target_dose` | Dose used for coverage classification. |
| `dose_tolerance_ratio` | Fractional tolerance around target dose. |
| `occlusion_enabled` | Enables front-surface occlusion. |
| `occlusion_cell_size` | Bin size for occlusion lookup. |
| `occlusion_depth_tolerance` | Allowed depth behind front surface. |
| `nozzle_offsets_*` | Per-nozzle local offsets. |
| `nozzle_axes_*` | Per-nozzle local spray directions. |
| `nozzle_wide_axes_*` | Per-nozzle local fan-wide directions. |

## Coverage Classification

For each canopy point:

```text
d_min = target_dose * (1 - dose_tolerance_ratio)
d_max = target_dose * (1 + dose_tolerance_ratio)
```

The point is classified as:

- undersprayed if `dose < d_min`
- well-sprayed if `d_min <= dose <= d_max`
- oversprayed if `dose > d_max`

## Notes and Limitations

- Live mode keeps accumulated dose voxels for history, while old live points are
  ignored for occlusion after `max_point_age`.
- Static mode stores only the first valid cloud snapshot.
- Dose is accumulated on downsampled cloud points, not on a mesh or continuous
  surface.
- The model is geometric and does not include droplet dynamics, air flow,
  nozzle pressure, leaf normals, or runoff.
- The dose units are relative model units unless calibrated against real spray
  measurements.

## Build

```bash
cd ros2_ws
colcon build --packages-select futuraps_spray_coverage
source install/setup.bash
```
