# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 (Jazzy) framework for a dual-arm robotic welding workcell: one arm grips
the workpiece while the other welds it. Proof-of-concept stage. This directory

## Packages (dependency order, roughly upstream to downstream)

- `hold_and_weld_description` — URDF/xacro robot models, meshes, controller configs, worlds
- `hold_and_weld_planning` (Python, ament_python) — weld seam extraction and path generation from CAD/mesh geometry, outputs JSON weld paths
- `hold_and_weld_gripper_sampler` (C++, ament_cmake) — constraint-aware grasp sampling on CAD surfaces, standalone node consuming YAML config
- `hold_and_weld_application` (C++, ament_cmake) — action servers, coordinator, and custom kinematics stack that execute the physical sequence
- `hold_and_weld_bringup` — launch files tying everything together

Each of `hold_and_weld_planning`, `hold_and_weld_gripper_sampler`, and
`hold_and_weld_application` has its own README with pipeline-level detail — read
the relevant one before making non-trivial changes in that package.
`hold_and_weld_planning` and `hold_and_weld_gripper_sampler` also have `PARAMS.md`
documenting every YAML config key.

`hold_and_weld_gripper_sampler` requires OpenCASCADE 7.9.3, CGAL, FCL, and Embree 4.
`hold_and_weld_application` requires MoveIt 2 and Google Ceres.
`hold_and_weld_planning` is pure Python (trimesh, manifold3d, scipy, pythonocc-core)
and needs no build step.

## Tests

C++ packages use `ament_add_gtest` (gripper_sampler test targets are auto-registered
from every `.cpp` file in `test/`; application lists its three targets explicitly in
`CMakeLists.txt`):

```bash
colcon test --packages-select hold_and_weld_gripper_sampler hold_and_weld_application
colcon test-result --verbose

# single test binary, after building:
./build/hold_and_weld_gripper_sampler/<test_name>
./build/hold_and_weld_application/<test_name>
```

`hold_and_weld_planning` (Python/ament_python) uses ament_flake8 + pydocstyle
(Google convention) + a copyright check as its "tests" — see
`test/test_flake8.py`, `test/test_copyright.py`.

Lint/style is also enforced repo-wide via pre-commit (`.pre-commit-config.yaml`):
ament_cpplint, ament_uncrustify (auto-reformats C++), ament_lint_cmake, ament_xmllint,
ament_flake8, pydocstyle, codespell. Run `pre-commit run --all-files` before
submitting non-trivial changes if the hooks are installed.

## Doxygen convention for private helpers

Public methods always get full `@brief` + one `@param` per parameter + `@return`
if non-void — that's the class's contract with everyone else, no exceptions.

Private/static helpers get `@brief` only, by default: the reader is already in
the `.cpp` file, one line above the body. Add `@param` to a private helper only
when the signature alone wouldn't stop a misuse the compiler can't catch:

- a boolean flag whose meaning isn't obvious from the name alone
  (`include_clearance` in `exclusion_zone_constraint.hpp` needs it; a name like
  `strict` would too — `hit_found` doesn't)
- an output/mutated parameter (non-const reference or pointer used to return
  data), e.g. the optional `resting_samples` output on
  `GroundConstraint::measure_ground_support`
- a unit, frame, or sign convention not encoded in the name (a bare `x`, `y`
  pair with no frame in the name)
- two or more same-type parameters whose order compiles either way but is
  silently wrong if swapped

Pick one or the other per file, not per function — a header with several
near-identical private helpers (e.g. a family of `parse_X(node, config)`
functions) reads fine as all-brief; a header of one-off helpers with
distinct signatures usually wants @param throughout.

## Cross-package data flow

```
hold_and_weld_planning  --weld path JSON-->   hold_and_weld_application (welder server)
hold_and_weld_gripper_sampler --grasp YAML--> hold_and_weld_application (gripper server)
hold_and_weld_description --URDF/SRDF/controllers-->  everything (MoveIt, controllers, action servers)
```

The planning and gripper-sampler packages are currently standalone/offline tools
(run via CLI, produce files) — they are not yet wired into the application layer
as live action servers (tracked in ROADMAP.md).

## `hold_and_weld_application` internals

- **Coordinator** (`src/coordinator`): temporary placeholder for behavior-tree
  orchestration. Only responsibility: trigger gripper server, wait for completion,
  then trigger welder server. Talks to action servers purely by ROS 2 action topic
  name (e.g. `trigger_gripper`, `trigger_welder`), never via MoveIt directly.
- **Action servers** (`src/action_servers`): `GripperActionServer` and
  `WelderActionServer`, both ROS 2 *lifecycle* action servers. Robot-agnostic —
  which physical robot/group they drive is entirely parameter-driven
  (`arm_group_name` etc. maps to a MoveIt planning group name from the SRDF).
  MoveIt 2 internals don't support lifecycle nodes, so both servers wrap an
  internal node-bridge workaround; expect noisy shutdown logs from MoveIt as a
  known, non-actionable issue. Gripper server runs a fixed 7-stage pipeline:
  Open → Approach → Pick → Close → Attach → Retract → Place.
- **Kinematics** (`src/kinematics`): a custom stack independent of MoveIt's IK,
  used specifically for *approach validation* without invoking the full MoveIt
  planning pipeline. `URDFParser` → `KinematicsSolver` (FK, Jacobian, Yoshikawa
  manipulability, joint limits) → `CeresIKSolver` (Ceres-based IK with warm
  starting/seed penalty to avoid joint flips) → `ApproachValidator` (walks a
  candidate approach configuration along a full weld seam, checking IK
  convergence and manipulability at each waypoint; rejects and lets OMPL replan
  on failure).

## Robot/prefix naming convention

Every robot instance is parameterized by a `prefix` (e.g. `robot1_`, `robot2_`)
threaded through URDF joint/link names, `controllers.yaml`, `moveit_controllers.yaml`,
the SRDF planning groups, and the relevant action server's launch-time parameters
(e.g. `arm_group_name` must match an SRDF group name exactly). See
`ADDING_A_ROBOT.md` for the full name-flow walkthrough and a cheat sheet of what
must be updated together when adding a robot, changing joint names, or adding a
controller/planning group/action server. The gripper and welder action servers
hardcode their frame names to the `robot1_`/`robot2_` slot prefixes by design —
those slots don't move, so swapping an arm model in place only requires a URDF
change (see `ADDING_A_ROBOT.md`), never a code or YAML change.

## `hold_and_weld_gripper_sampler` internals

C++ pipeline, single-threaded by design (PoC clarity): Geometry Mapper (loads
URDF/STEP, extracts OCCT topology) → Shape Refiner (splits U-periodic/large
surfaces, removes enclaves, heals geometry) → Contact Sampler (grid-based
antipodal contact pair search, filter/constraint-aware) → Angle Finder (4 phases:
radial ray classification → clustering/scoring → FCL/Embree collision checks →
self-collision checks). `Filters` (surface/region) and `Constraints`
(exclusion-zone, kissing-surface) are pluggable interfaces defined but not yet
pluginlib-ized — see ROADMAP.md. Constraints double as both sampling exclusions
and collision geometry during the Angle Finder phase.

## `hold_and_weld_planning` internals

Two independent seam extractors feeding a common `Core` object model
(`hold_and_weld_planning/core`: `Seam`, `LineSegment`, `ArcSegment`,
`PtPSegment`):

- **OCCT extractor** (`occt/`): CAD-exact, uses `BRepExtrema_DistShapeShape` for
  face-pair proximity and `BRepAlgoAPI_Common` for exact intersection edges.
  Geometry is exact and only two parameters are read (`epsilon`,
  `num_smooth_points`), but it is NOT free of judgement: which part supplies the
  base normal and which the wall is decided from boundary-edge topology, and a
  curve OCCT cannot name as a line or circle is demoted to a PtP segment. Pipe
  joint detection incomplete, and the package has no tests of its own.

- **Mesh extractor** (`mesh/`): contact-boundary based, pure Python. Marks faces
  within `epsilon` of the other mesh, takes the edges bounding that set, keeps
  those following a real part edge, chains them per mesh and stitches across
  meshes, then slides each point off the vertex lattice onto the half level set
  of a coverage field. Ownership — which mesh carries the edge — is decided per
  POINT, so it alternates along a chain wherever a part overhangs.
  Parameter-sensitive; `epsilon` is the central tolerance and is bounded above
  by `refine_iterations` (see PARAMS.md).
    - `mesh/params.py` — every tuning knob as a validated dataclass
    - `mesh/mesh_fields.py` — cached geometric queries, and the contact measure
    - `mesh/seam_extractor_mesh.py` — the extractor proper
    - `mesh/path_creator.py` — LINE/ARC/PTP classification

`planning/job_planner.py` and `planning/weld_planner.py` consume `Core` objects;
`weld_planner.py` is the standardized planner producing the JSON weld path
consumed by `hold_and_weld_application`'s welder server, auto-selecting
edge-to-edge vs edge-to-surface mode based on extractor output.
