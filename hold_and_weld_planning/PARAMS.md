Got it. Here is the Parameters.md:

---

# Parameters

## Workpiece

| Parameter | Type | Default | Description |
|---|---|---|---|
| `workpiece.mode` | string | `auto` | Processing mode. One of: `auto`, `mesh`, `occt` |
| `workpiece.auto_detect_seams` | bool | `false` | Enable automatic seam detection from geometry |
| `workpiece.main_part.main_path` | string | — | Path to main part. Supports URDF, xacro, STL, STEP, IGES and `package://` URIs. Required. |
| `workpiece.main_part.world_pose.xyz` | double[3] | [0, 0, 0] | Main part translation in world frame [m] |
| `workpiece.main_part.world_pose.rpy` | double[3] | [0, 0, 0] | Main part rotation as roll/pitch/yaw [rad] |
| `workpiece.secondary_part.secondary_path` | string | — | Path to secondary part. Supports same formats as main. Required. |
| `workpiece.secondary_part.world_pose.xyz` | double[3] | [0, 0, 0] | Secondary part translation in world frame [m] |
| `workpiece.secondary_part.world_pose.rpy` | double[3] | [0, 0, 0] | Secondary part rotation as roll/pitch/yaw [rad] |

## Planner

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.work_angle_deg` | double | — | Torch tilt angle perpendicular to travel direction [deg]. Required. |
| `parameters.travel_angle_deg` | double | — | Torch tilt angle along travel direction [deg]. Required. |
| `parameters.gap_mm` | double | — | Distance from seam to torch tip [mm]. Required. |
| `parameters.waypoint_spacing_mm` | double | 10.0 | Distance between generated waypoints along seam [mm] |
| `parameters.num_smooth_points` | int | 100 | Number of points per smoothed seam segment |

## Mesh

Parameters for the mesh-based seam extractor. Only used when `mode` is `mesh` or `auto` with mesh inputs.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.refine_iterations` | int | 32 | Mesh subdivision iterations for density increase before intersection |
| `parameters.inflate` | double | 1.002 | Scale factor applied to secondary mesh to create overlap for CGAL corefinement |
| `parameters.min_segment_length` | int | 5 | Minimum number of points for a chained path to be kept |
| `parameters.edge_detection_search_radius_ratio` | double | 0.05 | Search radius for edge detection as ratio of mesh bounding box size |
| `parameters.edge_detection_threshold` | double | 0.5 | Weighted edge score threshold above which a seam point is classified as edge contact |
| `parameters.gaussian_sigma_ratio` | double | 0.3 | Width of Gaussian weight window used in edge detection and normal outlier rejection |
| `parameters.normal_outlier_threshold_std` | double | 2.0 | Standard deviation threshold for normal outlier rejection |

## OCCT

Parameters for the OCCT-based seam extractor. Only used when `mode` is `occt` or `auto` with CAD inputs.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.epsilon` | double | 1e-3 | Distance tolerance for face contact detection and surface normal evaluation [m] |

## Path Creator

Optional tuning parameters for geometric path processing. Applied to mesh pipeline only.

### Outlier Removal

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.outlier_std_threshold` | double | 5.0 | Standard deviations above mean inter-point distance to classify a point as outlier |

### Corner Detection

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.min_sub_path_length` | int | 10 | Minimum points per sub-path after corner splitting |
| `parameters.min_points_for_corner_detection` | int | 20 | Minimum total points required to attempt corner detection |
| `parameters.corner_min_angle` | double | 25.0 | Minimum angle change to detect a corner [deg] |
| `parameters.corner_angle_window` | int | 10 | Window size for angle-based corner detection [points] |
| `parameters.corner_curvature_window` | int | 10 | Window size for curvature-based corner detection [points] |
| `parameters.corner_curvature_threshold` | double | 0.8 | Normalized curvature threshold for corner detection. Higher = sharper corners only |
| `parameters.corner_min_agreement` | int | 1 | Minimum number of detection methods that must agree on a corner |
| `parameters.corner_tolerance` | int | 5 | Maximum index distance between detections to group as same corner [points] |
| `parameters.corner_filter_window` | int | 5 | Window size for local maxima filtering of corner candidates [points] |

### Arc Merging

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.arc_radius_tolerance` | double | 0.05 | Maximum radius difference to merge consecutive arcs [m] |
| `parameters.arc_merge_num_points` | int | 100 | Number of points for smoothed merged arc |

### Line Merging

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.line_angle_tolerance_deg` | double | 10.0 | Maximum angle difference to merge consecutive lines [deg] |
| `parameters.line_merge_max_error` | double | 0.001 | Maximum fit error to allow line merge |
| `parameters.line_merge_max_iterations` | int | 20 | Maximum iterations for iterative line merging |

### Small Line Absorption

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.min_line_size` | int | 70 | Minimum line segment size to keep separate from adjacent arcs [points] |
| `parameters.line_absorption_max_error` | double | 0.01 | Maximum fit error to absorb a small line into an adjacent arc |
| `parameters.line_absorption_max_iterations` | int | 10 | Maximum iterations for absorption process |

### First/Last Segment Merging

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.first_last_angle_tolerance` | double | 10.0 | Angle tolerance for merging first and last line segments [deg] |
| `parameters.first_last_radius_tolerance` | double | 0.05 | Radius tolerance for merging first and last arc segments [m] |
| `parameters.first_last_gap_threshold` | double | 0.01 | Maximum spatial gap between first and last segment endpoints to allow merge [m] |

## Planned Parameters

These parameters exist in the codebase but are not yet fully exposed or stabilized.

| Parameter | Description |
|---|---|
| `parameters.epsilon` | Currently named inconsistently between `JobPlanner` (epsilon) and `SeamExtractor` (inflate/tolerance). Will be unified and properly namespaced in a future version. |
| `parameters.line_error_threshold` | Line fit error threshold. Not yet implemented or exposed to YAML configuration. |
| `parameters.circle_error_threshold` | Circle fit error threshold. Not yet implemented or exposed to YAML configuration. |
| `parameters.angle_threshold_deg` | Angle threshold in degrees. Not yet implemented or exposed to YAML configuration. |
