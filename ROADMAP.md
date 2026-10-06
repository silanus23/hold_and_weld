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
- **Mesh geometry support** — enabling sensor-driven workflows where CAD is unavailable.
  The planning pipeline already has a mesh seam extractor; the gripper sampler gets mesh
  support by integrating existing mesh and point-cloud samplers as selectable backends
  rather than building its own (see the gripper sampler's Mesh Support section).
- **Real hardware validation** — current validation is limited to simulation, on the
  GP25 pair and the HC10DT + GP8L cell. Systematic real hardware testing across
  supported configurations is planned.
- **Calibration helper tools** — tools that measure the real cell and write the results
  into the existing configs: the relative pose of the two robots (`workcell.yaml`), the
  torch and gripper TCPs, and the workpiece pose. Needed for the move from simulation to
  real hardware.
- **Mesh-based effectors and extra axes** — the system currently assumes parallel
  jaw grippers, and robot1's linear rail is the only extra axis. Integrating more
  realistic mesh-based effectors and extended extra axis support is planned as part of
  the robot-agnostic extensibility goal.

---

## AI-Based Improvements

- **Simulation-checked behavior trees** — a behavior tree composed or edited from an
  operator request (e.g. "weld everything except the underside seam") runs first in
  OmniSim, which an agent can drive through its MCP server, and reaches the robot only
  after passing there. The hand-written tree stays the default and the fallback. The
  tree nodes are the composer's vocabulary, so this needs parameterized action goals
  (seam IDs, grasp index), query services listing seams and grasps, explicit output IDs
  instead of the newest file in a folder, and reason codes in results.
- **Primary model decision nodes** — a primary model picks among the candidates the
  pipeline already ranks: the gripper sampler's grasps and the ConfigurationFinder's
  approach configurations. It is an optional backend behind a common interface; the ranked
  first candidate stays the default, and every choice still passes the deterministic
  checks before it reaches the robot.
- **Grasp dataset generation** — an output pipeline that takes the validated grasps of
  any sampler backend, CAD or external, and produces labeled grasp datasets. The
  practical motivation is that weld areas, screw holes, and similar features tend to
  repeat across workpieces in the same workplace. A generated dataset allows downstream
  models to learn avoidance of these regions without rerunning the full sampling
  pipeline on every new workpiece. This would also enable gripping strategies informed
  by spatial proximity to weld seams.

---

## Physical Simulation

Grasps are currently validated geometrically: contact, clearance and collision. Whether a
grasp physically holds is not checked. A physics-based validation stage is planned to
answer questions such as:

- Is the gripper's friction enough to hold the part against gravity and the accelerations
  of the holding arm's motion?
- Does the part stay put against the push of the welding torch and wire during welding?
- Does the part slip or rotate in the jaws, and by how much, under these loads?

Results would feed back into grasp scoring, so grasps that cannot physically hold are
rejected before execution. The planned simulator is OmniSim, using its MuJoCo physics
backend (through Newton). OmniSim provides STEP import, deterministic headless runs and
scene snapshots for checking each grasp candidate. The check needs per-surface friction
(gripper pad against part) and contact-force readback, which MuJoCo supports and OmniSim
does not yet expose. Thanks to the OmniSim team for reaching out.

---

## hold_and_weld_gripper_sampler

### Plugin System

- Pluginize constraint system (`ExclusionZoneConstraint`, `KissingSurfaceConstraint`,
  `GroundConstraint`) via pluginlib. There is no common base interface yet; one has to
  be extracted first.
- Pluginize filter system (`SurfaceFilter`, `RegionFilter`) via pluginlib.
  Base interfaces are already defined.
- Orientation grader — currently quality score is grippable arc fraction only.
  Add policy choices: force closure metric, approach clearance, robot joint reachability.

### Pipeline Improvements

- Concurrency. Pipeline is deliberately single threaded for proof of concept clarity.
  Contact pair processing and radial map construction are the primary parallelization
  targets.
- Dynamic sampling. Several sampling parameters are currently static. Adaptive density
  based on surface geometry and gripper dimensions is planned.
- Expose `FaceSamplingConfig::max_cells_per_tile` as a YAML key. Every caller uses the
  default of 16 today.
- Reproducible runs. The Angle Finder's randomization is seeded from the clock, so parts
  without symmetry (prism, elliptic part) give a slightly different candidate set on
  every run. Take the seed from the config, keeping the clock as the default.
- Guided sampling: identify geometrically promising sampling areas first to reduce
  brute-force surface traversal. Deferred together with the native mesh pipeline it was
  designed alongside.

### Validation

- Integration testing for the constraints beyond current unit tests, and for filters
  once they are wired.
- Real-life parts. Validated on primitive shapes and a complex non-convex part;
  real industrial parts may still reveal improvements.

### Action Server Integration

- Wire `hold_and_weld_gripper_sampler` into the `hold_and_weld_application` action server
  system via ROS 2 lifecycle action server. Reference implementations exist in
  `hold_and_weld_application`.

### Mesh Support

An in-house mesh sampler is deferred to keep development moving. The current goal is
integrability: making existing mesh and point-cloud grasp samplers (e.g. GPD) pluggable
alongside the CAD sampler, selectable per job from the config.

- A common sampler interface every backend implements, with the CAD sampler as the
  first implementation and the default when CAD is available.
- Adapters that convert each external sampler's parallel-jaw grasp output into the
  internal grasp format.
- External candidates pass through the same constraint checks (exclusion zones, ground,
  kissing surfaces), FCL collision check and scoring as CAD candidates, and are exported
  in the same JSON, so nothing downstream depends on which sampler produced them.
- External samplers are optional dependencies: the package builds and runs without them.

A native mesh pipeline may return later, if the integrated samplers prove insufficient
for industrial parts. The deferred design is two-phase: graph-based structural analysis
of the mesh (spanning-tree coverage, CGAL) to find graspable regions, then guided
breadth-first contact-pair sampling outward from promising regions, feeding the existing
contact pair interface.

### Code Quality

- Add `BRepCheck_Analyzer` validation to URDF geometry loading. STEP loading already
  validates.

---

## hold_and_weld_application

### Action Servers

- Worker thread watchdog timeout.
- Gripper settle from joint states instead of `finger_settle_sec`: after the controller
  reports success, wait until the finger velocities are near zero (covers both reaching
  the open limit and stalling on the part), with a timeout.
- `detach_object` wiring in `run_job` once re-grasp workflow is defined.
- ACM collision allowance per object instead of per link — required to handle complex
  multi-primitive objects correctly where per-link granularity is insufficient.
- Online parameter update for execution-time tunable parameters: velocity scaling,
  acceleration scaling, controller type.
- Make `move_to_pose` a lifecycle action server like the gripper and welder servers.
- Limit extra axis (robot1's rail) movement by parameters.
- Weld PtP seams as one blended Pilz sequence of the short line pieces they are divided
  into (see `hold_and_weld_planning`). They currently go through `computeCartesianPath`;
  separate Pilz motions would stop the torch between pieces.
- Planning scene coordinator to make scene management event driven.
- Seam-level resume for the welder: report completed seams in the result and accept an
  optional start/skip field in the goal, so an interrupted job continues with a new goal
  that re-approaches from standoff (a stop still ends the goal; no in-job pause).
- Manual weld rejection: an operator reviews the extracted seams and excludes any by
  seam ID before the welder runs them. Lets a person discard seams the extractors get
  wrong (incomplete pipe joints, false positives) instead of requiring perfect detection.

### Coordinator

- Deprecate coordinator in favor of behavior tree integration.
- Scene management via behavior tree nodes.

### Kinematics

- More test coverage on viable and non-viable seam paths.
- Lifecycle harmony with MoveIt 2 — currently requires architectural workarounds due to
  MoveIt 2's internal nodes not accepting lifecycle node interfaces.

---

## hold_and_weld_planning

The OCCT pipeline produces exact geometry and reads three parameters, but it is not
judgement-free (see the package README). Pipe joint detection remains incomplete and is
not planned for the near term; manual weld rejection (see `hold_and_weld_application`)
covers the seams it gets wrong.

Curves that are neither a line nor an arc come out of both extractors as `PtPSegment`s,
which Pilz cannot execute. They are to be divided into short line pieces Pilz can run
(see the welder item in `hold_and_weld_application`).

A separate planned capability is seam extraction from scanned mesh inputs where CAD
geometry is unavailable. This requires a different pipeline: plane segmentation from the
mesh, intersection computation between segmented planes, and convexity-based validation
to distinguish weld joints from internal mesh edges. This would extend the system to
sensor-driven workflows and real-world scanned geometry where the current CAD-dependent
pipelines cannot be used.
