# Adding a New Robot to the Hold-and-Weld Workcell

This guide explains exactly how robots are wired together in this project — from
URDF macro to action server — and walks you through adding a brand-new robot arm
as either a second gripper or a second welder (or anything else).
Note: Even though every 6 joint with needed files could be added,
system generally targets easiness on yaskawa motoman repo's conventions.

---

## How Names Flow Through the System

### 1. The `prefix` in URDF

Every robot in this project is instantiated with a **prefix string** (e.g.
`robot1_`, `robot2_`). This prefix is stamped onto every link and joint name:

```
prefix = "robot2_"
-> joints:  robot2_joint_1 … robot2_joint_6
-> links:   robot2_base_link, robot2_link_1 … robot2_link_6, robot2_tool0
-> ros2_control block name: robot2_gp25_system
```

All downstream config files must use these exact prefixed names.

### 2. `controllers.yaml` — joint names -> controller names

The controller config lists which URDF joint names each ROS 2 controller owns:

```yaml
# The controller_manager sees this block:
controller_manager:
  ros__parameters:
    robot2_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# The controller itself is configured here:
robot2_arm_controller:
  ros__parameters:
    joints:
      - robot2_joint_1   # <- must exactly match URDF joint names
      - robot2_joint_2
      ...
```

This creates the ROS 2 action topic:
`/robot2_arm_controller/follow_joint_trajectory`

### 3. `moveit_controllers.yaml` — controller names -> MoveIt

MoveIt is told which action topics to use when executing trajectories:

```yaml
robot2_arm_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory   # -> /robot2_arm_controller/follow_joint_trajectory
  joints:
    - robot2_joint_1
    ...
```

### 4. `dual_robot.srdf` — joints -> planning group

The SRDF groups joints (or defines a kinematic chain) into a named planning group:

```xml
<!-- Chain-based group (welder style) -->
<group name="robot2_welder_arm">
  <chain base_link="robot2_base_link" tip_link="robot2_wire_tip"/>
</group>

<!-- Joint-list group (gripper arm style) -->
<group name="robot1_arm">
  <joint name="robot1_rail_joint"/>
  <joint name="robot1_joint_1"/>
  ...
</group>
```

The group **name** (`robot2_welder_arm`, `robot1_arm`) is a plain string. It does
**not** have to contain the prefix, but by convention it does.

### 5. Action server — planning group name as a parameter

The action server receives the group name as a ROS 2 parameter at launch time and
passes it directly to MoveIt:

```cpp
// In on_configure():
arm_group_name_ = get_parameter("arm_group_name").as_string();  // e.g. "robot1_arm"
move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), arm_group_name_);
```

The launch file supplies this parameter:

```python
# app_gripper_server.launch.py
Node(
    ...
    parameters=[{
        'arm_group_name': 'robot1_arm',   # <- this string must match the SRDF group name
    }]
)
```

**This parameter defines robot name groups for action servers.** Change it and the server plans for a different robot.

> **Note:** The welder action server's kinematic-chain endpoints
> (`robot2_base_link`, `robot2_wire_tip`) and the gripper server's touch/attach
> links (`robot1_...`) are hardcoded to the `robot2_`/`robot1_` prefixes. This is
> intentional, not a gap: those prefixes are fixed per-slot in this system (there
> is exactly one gripper slot and one welder slot), so swapping which arm model
> occupies a slot — e.g. gp25 for ar2010 — only requires setting that slot's
> `model:` in `hold_and_weld_description/config/workcell.yaml` (by hand or with
> the workcell configurator), provided the model is registered in
> `urdf/robots/robot_catalog.xacro` (see "Registering a model in the robot
> catalog" below). Because every arm macro emits identical joint/link names for
> a given prefix (see below), no C++ change is needed for a model swap; only
> `joint_limits.yaml` needs that model's velocity/acceleration limits.

### 6. Coordinator — connects to action servers by topic name

The `DualRobotCoordinator` does not talk to MoveIt at all. It only holds action
clients that connect to the action server's topic:

```cpp
gripper_client_ = rclcpp_action::create_client<TriggerGripper>(this, "trigger_gripper");
welder_client_  = rclcpp_action::create_client<TriggerWelder>(this,  "trigger_welder");
```

The action server advertises on the same topic name:

```cpp
// In gripper_action_server.cpp on_activate():
action_server_ = rclcpp_action::create_server<TriggerGripper>(this, "trigger_gripper", ...);
```

Because both nodes live in the same ROS 2 namespace (`/`), the topics match
automatically. If you add a third robot with its own action server you must:

1. Give the new action server a **unique topic name** (e.g. `trigger_robot3`)
2. Add a matching client in the coordinator

---

## Step-by-Step: Adding a New Robot

The example below adds **robot3** — a second welding arm — as a complete walkthrough.
Substitute your own robot model, prefix, and role.

---

### Step 1 — Add the kinematic xacro macros

Location: `hold_and_weld_description/urdf/robots/`

**A. Pure kinematics file: `<model>_macro.xacro`**

This file contains only links, joints, and visual/collision geometry. It must
accept a `prefix` parameter so every name is unique:

```xml
<xacro:macro name="motoman_<model>" params="prefix">
  <link name="${prefix}base_link"> ... </link>
  <link name="${prefix}link_1"> ... </link>
  ...
  <joint name="${prefix}joint_1" type="revolute">
    <parent link="${prefix}base_link"/>
    <child  link="${prefix}link_1"/>
    ...
  </joint>
  ...
</xacro:macro>
```

**B. Assembly macro: `<model>_arm_prefix.xacro`**

This file adds the fixed mounting joint, calls the kinematic macro, and calls the
shared `robot_arm_ros2_control` macro:

```xml
<xacro:macro name="<model>_arm_macro"
             params="parent prefix x:=0 y:=0 z:=0 roll:=0 pitch:=0 yaw:=0
                     initial_pos_j1:=0 initial_pos_j2:=0 initial_pos_j3:=0
                     initial_pos_j4:=0 initial_pos_j5:=0 initial_pos_j6:=0
                     controller_config_file">

  <!-- Mount to parent -->
  <joint name="${prefix}base_joint" type="fixed">
    <parent link="${parent}"/>
    <child  link="${prefix}base_link"/>
    <origin xyz="${x} ${y} ${z}" rpy="${roll} ${pitch} ${yaw}"/>
  </joint>

  <!-- Kinematics -->
  <xacro:include filename="$(find hold_and_weld_description)/urdf/robots/<model>_macro.xacro"/>
  <xacro:motoman_<model> prefix="${prefix}"/>

  <!-- ros2_control hardware interface -->
  <xacro:include filename="$(find hold_and_weld_description)/urdf/robots/robot_arm_ros2_control.xacro"/>
  <xacro:robot_arm_ros2_control
    prefix="${prefix}"
    ros2_control_name="${prefix}<model>_system"
    initial_pos_j1="${initial_pos_j1}"
    initial_pos_j2="${initial_pos_j2}"
    initial_pos_j3="${initial_pos_j3}"
    initial_pos_j4="${initial_pos_j4}"
    initial_pos_j5="${initial_pos_j5}"
    initial_pos_j6="${initial_pos_j6}"/>
</xacro:macro>
```

> **Note:** `robot_arm_ros2_control.xacro` is the shared template that generates
> the `<ros2_control>` block with `initial_value` injected into each
> `<state_interface>`. You do not need to edit it.

### Registering a model in the robot catalog

The robot slots (`robot1`, `robot2`) don't call an arm macro directly: they call
`robot_arm_macro` from `urdf/robots/robot_catalog.xacro`, which dispatches on the
`model:` value in `config/workcell.yaml`. To make a new model selectable there
and in the workcell configurator's menus, edit `robot_catalog.xacro`:

1. `<xacro:include>` its `<model>_arm_prefix.xacro`.
2. Add the name to the `robot_catalog` list.
3. Add one `<xacro:if value="${model == '<model>'}">` branch calling
   `<model>_arm_macro` with the same arguments as the existing branches.

An unregistered `model:` fails at xacro time with the list of known models.
The configurator lists every `*_arm_prefix.xacro` it finds and leaves out (with
an error log) any that the catalog cannot render.

---


## Special Case: Adding a Second Gripper (Dual Gripper)

This section covers what changes if you want a second robot that uses the same
**gripper hardware and motion logic** as robot1.

### What is different from a plain robot3

`GripperActionServer`'s `touch_links_`/`attach_link_` are hardcoded to the
`robot1_` prefix (`robot1_tool0`, `robot1_link_6`, `robot1_flange`,
`robot1_gripper_base`, `robot1_left_finger`, `robot1_right_finger`). There is
only one gripper slot in this system, so this is not meant to be reconfigured —
a genuinely new, second, independently-controlled gripper robot needs its own
action server instance (a copy of `GripperActionServer` with its own hardcoded
prefix and its own node name/topic), not a parameter change.

If you're instead just swapping which arm model sits in the existing `robot1_`
slot (e.g. gp25 for ar2010), no action-server change is needed at all — see the
note in the previous section. That only works because every arm macro emits
identical joint/link names for a given prefix; verify your new macro does the
same (`<prefix>tool0`, `<prefix>link_6`, `<prefix>flange`, plus
`<prefix>gripper_base`, `<prefix>left_finger`, `<prefix>right_finger` from
`gripper_prefix.xacro`) before swapping it in.

### Additional URDF changes

You need a second gripper hardware block. In `dual_robot.xacro`:

```xml
<!-- Robot 3 is robot1-style: gp25 arm + gripper end effector -->
<xacro:gp25_arm_macro parent="world" prefix="robot3_" x="..." y="..." .../>
<!-- gripper_macro emits its own mount joint (robot3_tool0_to_gripper) -->
<xacro:gripper_macro  prefix="robot3_" parent="robot3_tool0"/>
```

### Additional controllers

`gripper_prefix.xacro` generates its own `<ros2_control>` block named
`${prefix}gripper_system`. You need a matching controller:

```yaml
robot3_gripper_controller:
  ros__parameters:
    joints:
      - robot3_left_finger_joint
      - robot3_right_finger_joint
    command_interfaces: [position]
    state_interfaces:   [position, velocity]
```

### SRDF additions

```xml
<group name="robot3_arm">
  <joint name="robot3_joint_1"/>
  ...
  <joint name="robot3_joint_6"/>
</group>

<group name="robot3_gripper">
  <joint name="robot3_left_finger_joint"/>
  <joint name="robot3_right_finger_joint"/>
</group>

<end_effector name="robot3_gripper_ee"
              parent_link="robot3_link_6"
              group="robot3_gripper"/>
```

### Launch file additions

A second, independently-controlled gripper is a copy of `GripperActionServer`
(new class/executable, e.g. `GripperActionServer3`) with `touch_links_`/
`attach_link_` hardcoded to the `robot3_` prefix, advertised under its own
action topic (e.g. `trigger_gripper3`). In `app_robot3_gripper_server.launch.py`:

```python
parameters=[{
    'arm_group_name':          'robot3_arm',
    'gripper_controller_topic': '/robot3_gripper_controller/follow_joint_trajectory',
    'gripper_joint_names':     ['robot3_left_finger_joint', 'robot3_right_finger_joint'],
}]
```

## Quick-Reference Cheat Sheet

```
When you change...             You must also update...
─────────────────────────────────────────────────────────────────────────
Joint names (URDF prefix)  ->  controllers.yaml (joints list)
                           ->  moveit_controllers.yaml (joints list)
                           ->  srdf (group joint list, if joint-based group)
                           ->  gripper_action_server hardcoded names (if applicable)

Add a new controller       ->  controllers.yaml (two sections)
                           ->  moveit_controllers.yaml (controller_names + block)
                           ->  controllers_spawn.launch.py (new spawner node)

Add a new planning group   ->  dual_robot.srdf (<group> tag)
                           ->  kinematics.yaml (solver entry)
                           ->  moveit_controllers.yaml (controller that owns those joints)

Change which robot an      ->  Launch file parameter:
action server controls         arm_group_name / welder_group_name

Add a new action server    ->  New launch file (app_<name>_server.launch.py)
                           ->  dual_robot_coordinator.cpp (new client + sequencing)
                           ->  system_bringup.launch.py (include new launch file)

Change spawn pose          ->  Task YAML (safety_pose.joint_positions)
                           ->  system_bringup.launch.py (xacro args build block)
                           ->  dual_robot.xacro (xacro:arg declarations)

Move a robot base / rail,  ->  config/workcell.yaml (or the workcell configurator:
or swap a slot's arm model     ros2 launch hold_and_weld_bringup
                               workcell_configurator.launch.py)
                           ->  joint_limits.yaml (new model's limits, on a swap)

Add an arm model           ->  urdf/robots/robot_catalog.xacro (include, list, branch)

Swap the gripper           ->  gripper_prefix.xacro only (geometry, limits, mount joint);
                               the gripper server reads finger limits from the URDF.
                               Set gripper.open_position in pick_place_targets.yaml
                               within them, or remove it to open fully.
```
