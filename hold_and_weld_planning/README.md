# hold_and_weld_planning

A weld path creator for 6 DOF robots. Built on OCCT, CGAL, manifold3d, and trimesh.
The current architecture assumes intersection lines between parts can be found reliably.
Weld joint type (butt, fillet, lap) does not affect the planner since it operates purely
on geometry outputs — any joint type is handled correctly given valid geometric values
from the seam extraction side. Outputs weld paths as `JSON`.

Regarding seam extractors: the OCCT-based extractor is more deterministic by nature of
exact CAD representations, while the mesh-based extractor remains sensitive to parameters.
Active investigation into academic sources is ongoing to improve its robustness.

## Core

The fundamental objects that seam extractors provide to the planner. This package exposes
these objects directly, allowing custom logic without rebuilding the full pipeline.

## Planner

Two planners are provided. The job planner is a classical base pipeline that provides
connection to each class. The weld planner is the standardized planner — its intake is
Core class objects and it produces standardized `JSON` outputs based on given parameters
and seam extractors. It has two modes — edge to edge and edge to surface — determined
automatically by the seam extractor output.

## Seam Extractors

### Mesh

Seam extractor for mesh inputs. Takes CGAL co-refinement as the starting point to get
intersection curves between meshes. Working with meshes requires probabilistic approaches.
Core objects need: contact type, which side of the intersection is surface or edge, and
normals around each point.

To extract those values the current approach chains CGAL intersection segments into
continuous paths using an adjacency graph, then applies aggressive corner detection
combining angle-based and curvature-based methods with agreement voting. Sub-paths are
split at detected corners, smoothed with B-splines, then classified as line or arc by
comparing fit errors. The better fit is always chosen regardless of absolute error
magnitude. A final merge pass tests compatibility between small segments and their
neighbors — consecutive arcs, consecutive lines, and small lines are absorbed into
adjacent larger segments where possible. Bimodality of normals around points is used to
determine contact type.

### OCCT

Seam extractor for CAD inputs. Uses exact face-pair proximity detection via
`BRepExtrema_DistShapeShape` to find kissing surfaces, then `BRepAlgoAPI_Common` to
extract exact intersection edges. For each intersection edge, wall surfaces are selected
based on whether a real boundary edge exists on the kissing face. Normals are evaluated
directly from OCCT surface properties at each point. The result is more deterministic
than the mesh pipeline with no parameter sensitivity on the geometry classification side.
Pipe joint detection is under development.

## Known Limitations

- Mesh pipeline remains sensitive to parameters. Results may vary depending on mesh
  density, inflation factor, and corner detection thresholds.
- Mesh path classification always picks the better of line or arc regardless of absolute
  fit error — no fallback for genuinely complex curves.
- Seam chaining does not handle points where three or more surfaces meet — additional
  branches are silently dropped.
- Pipe joint detection in the OCCT extractor is incomplete. Inner intersection curves
  are not distinguished from outer seam curves — users should verify output manually.
- Tested on box, plate, and cylinder workpieces. Complex organic geometry is not yet
  validated on either pipeline.
