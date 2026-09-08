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
| `parameters.refine_iterations` | int | 16 | Mesh subdivision iterations, applied per collision primitive |

`refine_iterations` is bounded from above by the contact test, per PART and set
by the THINNEST one: a wall row is `thickness/refine` high and its bottom
centroid sits at a third of that, so once `thickness/(3*refine) <= epsilon` the
wall is wrongly marked as contact and the seam collapses. At `epsilon` 2mm a
250mm wall breaks past refine 41.7; measured clean at 40, broken at 42. Raising
refinement requires lowering epsilon in step.

Note it subdivides each collision primitive, so parts do not end up equally
dense: at refine 40 a box (12 base faces) reaches 19,200 while a 128-segment
cylinder reaches 812,800.

### Contact boundary

The seam extractor proper. `epsilon` is the central tolerance here as well as
in OCCT mode; the rest are mesh-derived multipliers or candidate counts, so
they do not need changing when part size changes.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.epsilon` | double | 0.002 | Face is in contact when its centroid is within this of the other surface [m]. Also the fit-up gap the parts are allowed to stand off by. |
| `parameters.edge_angle_min_deg` | double | 0.0057 | Dihedral angle above which a mesh edge counts as a real part edge [deg]. Near zero by design, so a tessellated curved surface's facet edges all qualify. |
| `parameters.near_contact_edge_fraction` | double | 0.1 | How far off the other mesh a sharp edge may sit and still count as seam, as a fraction of this mesh's median edge length, on top of `epsilon`. Guards against welding a part's own edges away from the joint. |
| `parameters.stitch_gap_factor` | double | 3.0 | Endpoint gap below which two chain pieces are joined, as a multiple of the sampling density local to the two ends being joined. |
| `parameters.min_loop_points` | int | 4 | Fewest points that can constitute a loop. Also the window used for that end-local density. |
| `parameters.sharp_edge_candidates` | int | 32 | Sharp edges considered per query, taken by nearest midpoint, before exact point-to-segment distance. |
| `parameters.closest_face_candidates` | int | 12 | Faces considered per point when finding the nearest surface, taken by nearest centroid. |
| `parameters.closest_vertex_candidates` | int | 4 | Vertices whose incident faces are added to that candidate set, so a face whose centroid is far but whose body is near is not missed. |
| `parameters.eps_stability_factors` | list | [0.75, 1.5] | Multipliers on `epsilon` used to probe whether the contact boundary is stable against it. Must be positive and not 1.0. |
| `parameters.interpenetration_volume_m3` | double | 1e-12 | Intersection volume above which the parts are treated as interpenetrating rather than touching, which is out of scope [m^3]. |

#### Sub-vertex refinement

Seam points come out of the extractor as mesh vertices, so the seam is only as
fine as the tessellation and a contact boundary falling between two vertices is
lost. These control the field that recovers it: around each point the other
mesh's triangles are weighted by `area x (1-t^2)^3`, `t = d/rho`, and again by
how squarely each opposes the owner's own mating surface. Normalised by the
full-plane weight `pi*rho^2/4` that reads 1 inside the contact, 0 outside and
1/2 on the boundary, so the boundary is the half level set and can be solved
between vertices.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.kernel_radius_factor` | double | 1.0 | Kernel radius as a multiple of the local mean edge length, taken as `max(owner mesh, other mesh)` at the point |
| `parameters.coverage_bisection_steps` | int | 20 | Bisection steps used to land a point on the half level set |

`kernel_radius_factor` multiplies the **local mean** edge, not a global one:
global mean and median both understate the scale at the joint by 30-40% here,
and mean rather than median because at a vertex fan most incident edges are
short, so the median tracks the degenerate cluster (0.414mm against a mean of
2.314mm on this mesh). The `max` of the two meshes is required, not a
refinement: the coverage sum is a centroid-sampled integral over the *other*
mesh, and a radius below that mesh's triangle size caught zero centroids for
962 of 1829 points, leaving survival to be decided by whether a face centroid
happened to sit nearby.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.edge_joint_floor_factor` | double | 0.001 | How close a chain point must sit to the LOSING mesh's own sharp edge to count as edge-to-edge there, as a fraction of that mesh's median edge length. |
| `parameters.ownership_radius_factor` | double | 2.0 | Radius of the turning-density neighbourhood used to break a sharp-edge distance TIE, as a multiple of the coarser mesh's median edge length. |
| `parameters.ownership_tie_tolerance` | double | 1e-9 | Relative tolerance at which the two meshes' sharp-edge distances count as tied, and the density comparison takes over. |

`_owner` compares distance to each mesh's own nearest sharp edge, which is
informative wherever one part terminates and the other does not. Where the two
distances TIE it carries no information at all, and the comparison used to fall
through to `<=`, handing every tied point to whichever part was passed as
`mesh_1`. That made the output depend on config order: measured 8 owner flips
on a saddle at refine=16, every one at `d1 == d2 == 0.0000mm`, which
`PathCreator._split_on_contact_type` then turns into extra seam segments.

The tie is not a coincidence. On a curved mating surface the tessellation's own
facet seams clear `edge_angle_min` too, so both meshes report a sharp edge
underfoot. Ties are broken on turning density instead - `sum(dihedral *
edge_length) / sum(face area)` over a ball, measured on each mesh separately -
which is comparative in the same way the distance is, with no absolute angle
anywhere. Measured: flat reads 0, a cylinder of radius r reads 1/r at ANY
tessellation, a sharp crease reads `pi/(4*radius)`, a fillet of radius r_f
reads about 1/r_f. On the reproduced tie (coarse pipe through a plate with a
round hole) the pipe reads 0.1011 against a predicted 1/r = 0.100 and the plate
0.354, so the plate wins 15/15 tied points and the swap test goes to **0 owner
flips**. Points where distance already decides are untouched.

`ownership_radius_factor` is bounded below by the coarser mesh's facet size -
under that there is nothing to measure. It also has an upper bound: a crease's
density falls as `pi/(4*radius)` while a curved wall's stays at 1/r, so the two
cross at radius ~ `0.79*r` and looking WIDER weakens the discrimination. The
default sits just above the resolution floor for that reason. The working band
is `local facet size < radius < 0.79 * radius of the curved side`; on a mesh
coarse enough to close that band the question is undecidable at any radius.

The density is an integral over an area, not a per-edge `angle / width` ratio.
That is deliberate: coplanar refinement (`refine_iterations`) does not move a
facet seam or change its dihedral, it only narrows the strip around it, so a
per-edge form inflates without bound under refinement and eventually ranks a
smooth wall above a real edge (measured 0.151 -> 19.3 on a cylinder while a
real crease at comparable mesh size read 5.3). Both sums here are conserved
under retriangulation instead - measured 0.5% drift across a 60x face count
increase - which also makes it robust to sliver triangles, since a sliver
contributes a small area and only its own real turning.

`_owner` is already threshold-free: it compares which mesh's sharp edge is
nearer, and edge-to-edge is where that comparison is effectively a tie - both
parts terminating on the same curve. `edge_joint_floor_factor` floors the
LOSER's distance against its own mesh's tessellation scale rather than
`epsilon`, because this residual is a corefinement/refine artifact, two to
three orders of magnitude smaller than the fit-up gap `epsilon` is sized for.
Measured on a real butt-joint part: loser distance min=0.000/median=0.001/
p95=0.003/max=0.003mm over a 158-point chain whose mesh has a ~12.5mm median
edge, well inside the 0.0125mm floor the default factor gives there.

Keep the factor small. Measured position error against analytic: k=1 -> 0.07mm,
k=2 -> 0.54mm, k=4 -> 1.85mm, as a larger neighbourhood starts seeing the rim's
curvature and the local half-plane assumption breaks.

This improves with refinement, unlike the vertex lattice it replaces: measured
error falls 4.31mm -> 0.065mm going from `refine_iterations` 16 -> 40, because
the level set is a better approximation of the true rim the more triangles
describe it. Below about refine 20 it is *worse* than snapping to vertices, and
where the mesh is too coarse for the half crossing to bracket at all the point
is left on its vertex, so a coarse part degrades to the old behaviour.

## OCCT

Parameters for the OCCT-based seam extractor. Only used when `mode` is `occt` or `auto` with CAD inputs.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.epsilon` | double | 1e-3 | Distance tolerance for face-pair proximity [m]. Shared with mesh mode, see Contact boundary above. |

## Path Creator

Classifies the ordered seam points into LINE, ARC and PTP segments. Every
parameter below is read by `mesh/path_creator.py`; all are optional.

The cascade, in order: a run is a LINE if a straight line holds
`path_tolerance_mm`; an ARC if a circle holds the stricter
`arc_strictness x path_tolerance_mm` AND subtends at least `min_arc_angle_deg`
AND consumes `arc_gain` times the run a line would; otherwise PTP. The
asymmetry is deliberate - a false PTP only densifies waypoints, a false arc
leaves the seam.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `parameters.path_tolerance_mm` | double | 1.0 | Master fit tolerance: max deviation from a fitted line [mm] |
| `parameters.arc_strictness` | double | 0.5 | Arc tolerance as a fraction of `path_tolerance_mm`. 0 disables arcs. |
| `parameters.min_arc_angle_deg` | double | 15.0 | Below this subtended angle a run is not worth calling an arc [deg] |
| `parameters.arc_gain` | double | 1.5 | An arc must consume this multiple of the run a line would, or the line wins |
| `parameters.min_fit_points` | int | 4 | Fewest points a segment may be fitted from. Must be >= 3. |
| `parameters.max_line_length` | double | 0.5 | Hard split length for a line, not geometric [m] |
| `parameters.max_arc_length` | double | 0.5 | Hard split length for an arc [m]. Why a 706.79mm semicircle emerges as two arcs of 353.39mm. |
| `parameters.max_ptp_length` | double | 0.1 | Hard split length for a PTP run [m] |

`waypoint_spacing_mm` (see Planner) is also read here: a run too short to carry
two weld poses is not a segment, so the process spacing doubles as the minimum
length of a joint-character run before it is absorbed into its neighbour.

## Planned Parameters

These parameters exist in the codebase but are not yet fully exposed or stabilized.

| Parameter | Description |
|---|---|
| `parameters.epsilon` | One key serves two jobs in mesh mode: the contact test in `_boundary_edges` and the fit-up gap. Splitting them is open work. |
| `parameters.line_error_threshold` | Line fit error threshold. Not yet implemented or exposed to YAML configuration. |
| `parameters.circle_error_threshold` | Circle fit error threshold. Not yet implemented or exposed to YAML configuration. |
| `parameters.angle_threshold_deg` | Angle threshold in degrees. Not yet implemented or exposed to YAML configuration. |
