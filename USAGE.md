# Usage

Every launch file takes `--show-args` to list its arguments and their defaults.

## Simulation

```bash
ros2 launch hold_and_weld_bringup system_bringup.launch.py
```

Both robots: the gripper arm picks the part, then the welder arm welds it.

- `auto_start:=false`: start without running the job.
- `use_gazebo_gui:=false`, `use_rviz:=false`: run without those windows.

```bash
ros2 launch hold_and_weld_bringup gripper_bringup.launch.py
```

Gripper arm only: picks and places the part.

- `auto_trigger:=false`: start without running the job.
- `hold_and_weld_bringup/config/tasks/pick_place_targets.yaml`: finger positions,
  pick and place poses.

```bash
ros2 launch hold_and_weld_bringup welder_bringup.launch.py
```

Welder arm only: welds the newest path in `hold_and_weld_application/trajectories/`.

- `auto_trigger:=false`: start without running the job.
- `hold_and_weld_bringup/config/tasks/welding.yaml`: safety pose, weld motion
  settings, `json_file` to weld one specific path.

## Visualization (RViz only, no Gazebo)

```bash
ros2 launch hold_and_weld_bringup magic_wand.launch.py
```

Shows the newest weld path.

```bash
ros2 launch hold_and_weld_bringup finger_visualizer.launch.py
```

Shows the newest grasps from the sampler.

- `max_grasps:=N`: show only the first N.

## Workcell setup

```bash
ros2 launch hold_and_weld_bringup workcell_configurator.launch.py
```

Place the robots and parts in RViz. Saves go to
`hold_and_weld_bringup/config/configurator_output/`; copy them over `workcell.yaml`
and `objects.yaml` to use them.

## Offline tools

```bash
ros2 run hold_and_weld_planning seam_generator -i <config.yaml>
```

Finds weld seams and writes a path to `hold_and_weld_application/trajectories/`.

- `hold_and_weld_planning/config/urdf_welding_conf.yaml`: the default config; parts,
  poses, extractor settings (see `PARAMS.md`).

```bash
ros2 run hold_and_weld_gripper_sampler grasp_finder_node --config <config.yaml>
```

Finds grasps and writes them to `hold_and_weld_application`'s installed
`grasps/grasps.json`.

- `hold_and_weld_gripper_sampler/config/grasp_finder_*.yaml`: example configs;
  part, gripper, exclusion zones (see `PARAMS.md`).

## Shared YAML files

- `hold_and_weld_description/config/workcell.yaml`: robot models and positions,
  gripper model.
- `hold_and_weld_bringup/config/objects/objects.yaml`: part models and positions,
  in Gazebo and in the planning scene.
- `hold_and_weld_bringup/config/moveit/`: joint limits, weld speed, IK and planner
  settings.
- `hold_and_weld_description/config/*controllers.yaml`: ros2_control controllers.

## Building blocks

The other launch files (`app_*`, `controllers_spawn`, `moveit_*`, `sim_gazebo`,
`sim_spawn_objects`) are parts of the launches above. Start one alone only to
restart it on a running system.
