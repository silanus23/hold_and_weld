```markdown
# Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `frame_id` | string | `world` | Coordinate frame ID attached to all output grasp poses |

## Primary

| Parameter | Type | Default | Description |
|---|---|---|---|
| `primary.urdf_path` | string | — | Path to workpiece URDF file. Either this or `step_path` required. |
| `primary.step_path` | string | — | Path to workpiece STEP file. Either this or `urdf_path` required. |
| `primary.transform.translation.x` | double | 0.0 | Workpiece X translation in world frame [m] |
| `primary.transform.translation.y` | double | 0.0 | Workpiece Y translation in world frame [m] |
| `primary.transform.translation.z` | double | 0.0 | Workpiece Z translation in world frame [m] |
| `primary.transform.rotation.x` | double | 0.0 | Workpiece rotation quaternion X |
| `primary.transform.rotation.y` | double | 0.0 | Workpiece rotation quaternion Y |
| `primary.transform.rotation.z` | double | 0.0 | Workpiece rotation quaternion Z |
| `primary.transform.rotation.w` | double | 1.0 | Workpiece rotation quaternion W |

## Gripper

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gripper.urdf_path` | string | — | Path to gripper URDF file. Required. Must contain `<gripper_metadata>` section. |
| `gripper.max_opening` | double | from URDF | Override maximum gripper opening [m]. If not set, uses joint limits from URDF. |

## Secondaries

Secondaries are defined as a sequence. Each secondary must have a `type` field.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `secondaries[].id` | string | — | Unique identifier for this secondary |
| `secondaries[].type` | string | — | Shape type. One of: `ground_plane`, `box`, `cylinder`, `step`, `urdf` |
| `secondaries[].transform.translation` | x/y/z doubles | 0.0 | Translation in world frame [m]. Not applicable for `ground_plane`. |
| `secondaries[].transform.rotation` | x/y/z/w doubles | identity | Rotation quaternion. Not applicable for `ground_plane`. |

**Type: `ground_plane`**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `secondaries[].size_x` | double | 2.0 | Ground plane extent in X [m] |
| `secondaries[].size_y` | double | 2.0 | Ground plane extent in Y [m] |
| `secondaries[].z_position` | double | 0.0 | Z position of ground plane surface [m] |

**Type: `box`**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `secondaries[].dimensions.x` | double | — | Box X dimension [m] |
| `secondaries[].dimensions.y` | double | — | Box Y dimension [m] |
| `secondaries[].dimensions.z` | double | — | Box Z dimension [m] |

**Type: `cylinder`**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `secondaries[].radius` | double | — | Cylinder radius [m] |
| `secondaries[].height` | double | — | Cylinder height [m] |

**Type: `step` / `urdf`**

| Parameter | Type | Default | Description |
|---|---|---|---|
| `secondaries[].step_path` | string | — | Path to STEP file. Required for `step` type. |
| `secondaries[].urdf_path` | string | — | Path to URDF file. Required for `urdf` type. |

## Exclusion Zones

### Circles

| Parameter | Type | Default | Description |
|---|---|---|---|
| `exclusion_zones.circles[].id` | string | — | Unique identifier |
| `exclusion_zones.circles[].center.x` | double | — | Circle center X [m] |
| `exclusion_zones.circles[].center.y` | double | — | Circle center Y [m] |
| `exclusion_zones.circles[].center.z` | double | — | Circle center Z [m] |
| `exclusion_zones.circles[].normal.x` | double | — | Surface normal X |
| `exclusion_zones.circles[].normal.y` | double | — | Surface normal Y |
| `exclusion_zones.circles[].normal.z` | double | — | Surface normal Z |
| `exclusion_zones.circles[].radius` | double | — | Exclusion radius [m] |
| `exclusion_zones.circles[].projection_depth` | double | — | Depth to project volume into surface [m] |
| `exclusion_zones.circles[].clearance` | double | 0.01 | Additional safety margin [m] |

### Lines

| Parameter | Type | Default | Description |
|---|---|---|---|
| `exclusion_zones.lines[].id` | string | — | Unique identifier |
| `exclusion_zones.lines[].start.x/y/z` | double | — | Line start point [m] |
| `exclusion_zones.lines[].end.x/y/z` | double | — | Line end point [m] |
| `exclusion_zones.lines[].exclusion_radius` | double | — | Tube radius around line [m] |
| `exclusion_zones.lines[].clearance` | double | 0.01 | Additional safety margin [m] |

### Polygons

| Parameter | Type | Default | Description |
|---|---|---|---|
| `exclusion_zones.polygons[].id` | string | — | Unique identifier |
| `exclusion_zones.polygons[].corners` | list of x/y/z | — | Polygon corner points. Minimum 3. [m] |
| `exclusion_zones.polygons[].projection_depth` | double | — | Depth to project volume into surface [m] |
| `exclusion_zones.polygons[].clearance` | double | 0.01 | Additional safety margin [m] |

## Sampling

| Parameter | Type | Default | Description |
|---|---|---|---|
| `sampling.min_angle_deg` | double | 160.0 | Minimum angle between opposing surface normals [deg] |
| `sampling.max_angle_deg` | double | 180.0 | Maximum angle between opposing surface normals [deg] |
| `sampling.min_gripper_opening` | double | 0.02 | Minimum valid grip distance [m] |
| `sampling.max_gripper_opening` | double | 0.15 | Maximum valid grip distance [m] |
| `sampling.sample_density` | double | 0.01 | Grid spacing for surface sampling [m] |
| `sampling.normal_sample_density` | double | 1.0 | Samples per cm² for normal antiparallelism check |
| `sampling.alignment_threshold` | double | 0.95 | Minimum dot product between grip axis and surface normal |
| `sampling.max_lateral_deviation` | double | 0.02 | Maximum allowed lateral offset between contact points [m] |

## Orientation

| Parameter | Type | Default | Description |
|---|---|---|---|
| `orientation.finger_length` | double | 0.10 | Finger length, defines outer ring radius for radial map [m] |
| `orientation.finger_radius` | double | 0.02 | Finger radius, defines inner ring radius for radial map [m] |
| `orientation.max_orientations_per_pair` | size_t | 0 | Maximum orientations per contact pair. 0 = unlimited |
| `orientation.collision_tolerance` | double | 0.001 | Collision tolerance for approach direction validation [m] |
| `orientation.stop_on_first_valid` | bool | false | Stop searching after first valid orientation found |
| `orientation.ring_step_size` | double | 0.010 | Radial step between rings [m] |
| `orientation.angular_step_deg` | double | 2.0 | Angular step for ray casting in radial map [deg] |
| `orientation.flat_detection_tolerance_m` | double | 0.003 | Tolerance for classifying a ray hit as flat vs elevated [m] |
| `orientation.cliff_merge_tolerance_deg` | double | 2.0 | Angular tolerance for merging adjacent cliff segments [deg] |
| `orientation.min_cliff_width_deg` | double | 5.0 | Minimum arc width for a cliff segment to be considered valid [deg] |
| `orientation.ray_lift_offset` | double | 0.010 | Height above contact point at which radial rays are cast [m] |
| `orientation.seed_step_deg` | double | 15.0 | Angular step between seed candidates within a valid LOW arc [deg] |
| `orientation.randomize_seeds` | bool | false | Randomize seed angles within valid arcs instead of uniform grid |
| `orientation.max_edge_candidates` | size_t | 3 | Maximum edge candidates per contact point. Kept for config compatibility. |
| `orientation.dual_seed_dedup_tolerance_deg` | double | 3.0 | Kept for config compatibility. |
| `orientation.max_edges_per_contact` | size_t | 0 | Kept for config compatibility. |
| `orientation.angle_offsets` | double[] | [-15.0, 0.0, 15.0] | Kept for config compatibility. |

### Debug

| Parameter | Type | Default | Description |
|---|---|---|---|
| `orientation.debug_full_sweep` | bool | false | Bypass radial map entirely and test all angles at uniform step. Use to diagnose collision checker behaviour across full 360°. |
| `orientation.debug_sweep_step_deg` | double | 10.0 | Angular step for debug full sweep [deg] |

## Shape Refiner

| Parameter | Type | Default | Description |
|---|---|---|---|
| `shape_refiner.enabled` | bool | true | Enable shape refinement |
| `shape_refiner.max_cylinder_radius` | double | 0.100 | Cylinders with radius below this are split by arc length only. Above this triggers radius based split [m] |
| `shape_refiner.max_arc_length` | double | 0.200 | Maximum allowed edge arc length before splitting [m] |
| `shape_refiner.enclave_area_ratio` | double | 0.005 | Maximum enclave area as fraction of total shape area before suppression |
| `shape_refiner.enclave_angle_threshold` | double | 45.0 | Maximum wall angle for enclave suppression [deg]. Walls steeper than this are kept as real features. |

## Kissing

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kissing.contact_threshold` | double | 0.8 | Surfaces with contact area ratio above this are banned from sampling |
| `kissing.contact_distance_threshold` | double | 0.005 | Maximum distance between primary and secondary surfaces to classify as kissing contact [m] |
| `kissing.collision_tolerance` | double | 1e-6 | Distance threshold for secondary collision detection [m] |

## FCL

| Parameter | Type | Default | Description |
|---|---|---|---|
| `fcl.enabled` | bool | true | Enable FCL collision checking. Required for sampling. |
| `fcl.triangulation_deflection` | double | 0.0001 | Triangulation chord-height tolerance for FCL mesh generation [m] |

## Mesh Deflection

Controls OCCT triangulation quality for exclusion zone geometry.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mesh_deflection.linear` | double | 0.001 | Maximum distance between mesh edge and actual curve [m] |
| `mesh_deflection.angular` | double | 0.1 | Maximum angular deviation between adjacent mesh triangles [rad] |

## Output

| Parameter | Type | Default | Description |
|---|---|---|---|
| `output.json_path` | string | grasps.json | Output JSON file path |
| `output.max_grasps` | size_t | 0 | Maximum grasps to output. 0 = all |
| `output.min_quality` | double | 0.0 | Minimum quality score threshold for output |

## Planned Parameters

These parameters exist in the codebase but are not yet exposed to YAML configuration.
They will become user facing in future versions.

| Parameter | Description |
|---|---|
| `ground_normal_z_threshold` | Auto-detection of ground-facing surfaces from workpiece topology |
| `ground_safety_margin` | Z offset added to ground plane to avoid false positives at contact plane |
| `ground_size_x` | Ground plane size in X for kissing surface contact analysis |
| `ground_size_y` | Ground plane size in Y for kissing surface contact analysis |
```

A few things to check before approving:

- `max_edge_candidates`, `dual_seed_dedup_tolerance_deg`, `max_edges_per_contact`, `angle_offsets` — I marked these as kept for config compatibility since the radial map replaced the edge-based approach. Are these still meaningful or should they be marked deprecated?
- Default for `max_orientations_per_pair` — code shows 0 as default but `find_valid_grasps` uses 16 as internal fallback when 0 is set. Worth noting?
- Anything missing or wrong?
