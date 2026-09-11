# Roadmap

Items are not ordered by priority. Priority will be determined by integration
requirements as the system matures.

---

## System

- **Package names and namespaces** — current naming is verbose. Shorter, consistent
  naming across packages, namespaces, and topics is planned.
- **OCCT 8 transition** — currently built against OCCT 7.9.3.
  Transitioning to OCCT 8 is planned to take advantage of API improvements and
  long-term support.
- **Test coverage** — unit and integration test coverage needs to be increased
  systematically across all packages.
- **Behavior tree integration** — the coordinator is a temporary placeholder.
  Full behavior tree based task orchestration is the system-level end goal,
  replacing the coordinator and enabling conditional, multi-step, arbitrary-robot
  scenes.
- **Mesh geometry support** — both the gripper sampler and planning pipeline have
  mesh support as independent goals. At the system level this is a unified capability
  milestone enabling sensor-driven workflows where CAD is unavailable.
- **Real hardware validation** — current validation is limited to simulation and
  GP25 geometry. Systematic real hardware testing across supported configurations
  is planned.
- **Mesh-based effectors and extra axes** — the system currently assumes parallel
  jaw grippers and fixed-base robots. Integrating more realistic mesh-based effectors
  and extended extra axis support is planned as part of the robot-agnostic extensibility
  goal.

---

## hold_and_weld_gripper_sampler

### Plugin System

- Pluginize constraint system (`ExclusionZoneConstraint`, `KissingSurfaceConstraint`)
  via pluginlib. Base interfaces are already defined.
- Pluginize filter system (`SurfaceFilter`, `RegionFilter`) via pluginlib.
  Base interfaces are already defined.
- Orientation grader — currently quality score is grippable arc fraction only.
  Add policy choices: force closure metric, approach clearance, robot joint reachability.

### Pipeline Improvements

- Smart ground rejection. Current halfspace model is geometry agnostic. Replace with
  topology aware detection using workpiece surface normals and z-position analysis.
- Concurrency. Pipeline is deliberately single threaded for proof of concept clarity.
  Contact pair processing and radial map construction are the primary parallelization
  targets.
- Dynamic sampling. Several sampling parameters are currently static. Adaptive density
  based on surface geometry and gripper dimensions is planned.
- OCCT pipeline can benefit from mesh support techniques to identify geometrically
  fertile sampling areas more efficiently, reducing brute-force surface traversal.

### Validation

- Integration testing for filters and exclusion zone constraints beyond current unit tests.
- Organic shape testing. Current validation covers box, prism, and small cylinder only.
  Complex non-convex geometry is not yet validated.

### Action Server Integration

- Wire `hold_and_weld_gripper_sampler` into the `hold_and_weld_application` action server
  system via ROS 2 lifecycle action server. Reference implementations exist in
  `hold_and_weld_application`.

### Mesh Support

Mesh based geometry input as an alternative to CAD for sensor driven or CAD unavailable
scenarios. Planned as a two-phase pipeline.

Phase one performs structural analysis of the mesh — identifying graspable regions and
natural surface boundaries through graph based traversal. A spanning tree based approach
is used to guarantee complete mesh coverage and handle topologically circular regions
where no natural boundary exists. CGAL is the targeted library given existing dependency
availability in the system.

Phase two generates ranked contact pairs through guided sampling, starting from
geometrically promising regions and propagating outward via breadth-first search. Surface
boundaries are classified as either geometric features or traversal constructs, kept
strictly separate so ranking reflects only real geometry. Output feeds into the existing
contact pair interface with no downstream changes required.

This is a significant architectural addition planned after core CAD pipeline stabilization.

### Dataset Generation

An output pipeline that takes mesh inputs and produces labeled grasp datasets. The
practical motivation is that weld areas, screw holes, and similar features tend to repeat
across workpieces in the same workplace. A generated dataset allows downstream models to
learn avoidance of these regions without rerunning the full sampling pipeline on every
new workpiece. This would also enable gripping strategies informed by spatial proximity
to weld seams.

### Code Quality

- Add `BRepCheck_Analyzer` validation at geometry entry points for robustness on external
  STEP files.
- Fix known error handling inconsistencies per error handling policy. Primarily single
  layer catch blocks in `contact_point_sampler.cpp` and unguarded `UIso`/`VIso` handle
  checks in `shape_refiner.cpp`.

---

## hold_and_weld_application

### Action Servers

- Worker thread watchdog timeout.
- `detach_object` wiring in `run_job` once re-grasp workflow is defined.
- ACM collision allowance per object instead of per link — required to handle complex
  multi-primitive objects correctly where per-link granularity is insufficient.
- `YAML` value validation in `load_job_from_yaml`.
- Online parameter update for execution-time tunable parameters: velocity scaling,
  acceleration scaling, controller type.
- Replace `move_to_pose` with proper implementation.
- Limit extra axis movement by parameters.
- Planning scene coordinator to make scene management event driven.

### Coordinator

- Multi-planner pipeline: OMPL for collision-free approach planning, Pilz Industrial
  Motion Planner for sharp deterministic path execution — cleaner separation of concerns
  than the current single-planner approach.
- Deprecate coordinator in favor of behavior tree integration.
- Scene management via behavior tree nodes.

### Kinematics

- More test coverage on viable and non-viable seam paths.
- Lifecycle harmony with MoveIt 2 — currently requires architectural workarounds due to
  MoveIt 2's internal nodes not accepting lifecycle node interfaces.
- Approach configuration finder — derive a guaranteed-valid joint goal from the
  validator's IK walk, feed directly to OMPL as goal state, eliminating the current
  retry loop entirely.

---

## hold_and_weld_planning

The OCCT pipeline produces exact geometry and reads only two parameters, but "stable"
overstated it: several silent-output defects were found and fixed by hand-built probes,
including normals reversed on any seam wrapping past 180 degrees and unnamed curves
exported as zero-length lines. The package still has **no tests of its own**, which is
why those needed probes to find, and that is the first thing to address here. Pipe joint
detection remains incomplete and is not planned for the near term.

The mesh pipeline's fundamental problem used to be that CGAL corefinement pinned
intersection-segment endpoints to triangle edges rather than the true intersection
curve. That pipeline has been retired. The contact-boundary extractor that replaced it
never computes an intersection curve at all — it locates the boundary of the contact
region and then slides each point off the vertex lattice onto the half level set of a
coverage field, which resolves the sub-vertex problem directly and, unlike the lattice
it replaced, improves with refinement (measured 4.31mm -> 0.065mm going from
`refine_iterations` 16 to 40).

Of the two candidate fixes previously listed, ridge-valley detection was implemented and
retired, and Newton refinement was not needed once the level-set approach worked.

The core primitive planned here for complex curves — those not adequately represented as
sequences of line and arc segments — now exists as `PtPSegment`, and both extractors emit
it: the mesh path classifier demotes any run that fails the tolerance cascade, and the
OCCT extractor uses it for any curve OCCT cannot identify as a line or a circle.

A separate planned capability is seam extraction from scanned mesh inputs where CAD
geometry is unavailable. This requires a different pipeline: plane segmentation from the
mesh, intersection computation between segmented planes, and convexity-based validation
to distinguish weld joints from internal mesh edges. This would extend the system to
sensor-driven workflows and real-world scanned geometry where the current CAD-dependent
pipelines cannot be used.
