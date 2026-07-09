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
`hold_and_weld_planning`'s mesh pipeline needs a manually built extension:

```bash
cd hold_and_weld_planning/hold_and_weld_planning/mesh
mkdir build && cd build && cmake .. && make
cp mesh_intersection*.so ../
```

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
controller/planning group/action server. Note: the welder action server still has
some hardcoded robot-specific names (not yet parameterized like the gripper server).

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
(`hold_and_weld_planning/core`: `Seam`, `LineSegment`, `ArcSegment`):

- **OCCT extractor** (`occt/`): CAD-exact, uses `BRepExtrema_DistShapeShape` for
  face-pair proximity and `BRepAlgoAPI_Common` for exact intersection edges.
  Deterministic, no parameter sensitivity. Pipe joint detection incomplete.

- **Mesh extractor** (`mesh/`): CGAL corefinement-based, requires the compiled
  `mesh_intersection` C++ extension (see Build). Probabilistic/parameter-sensitive
  — chains intersection segments via adjacency graph, corner-detects, B-spline
  smooths, classifies line-vs-arc by best fit. Known limitation: corefinement
  vertices are pinned to triangle edges, not the true intersection curve (see
  ROADMAP.md for the two candidate fixes).

`planning/job_planner.py` and `planning/weld_planner.py` consume `Core` objects;
`weld_planner.py` is the standardized planner producing the JSON weld path
consumed by `hold_and_weld_application`'s welder server, auto-selecting
edge-to-edge vs edge-to-surface mode based on extractor output.
