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
-> ros2_control block name: robot2_ar2010_system
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

> **Note:** The welder action server currently has some hardcoded robot-specific values such as link names and tool frame references. This is a temporary compromise and will be parameterised in the same way as the gripper server. Adding a second welder before that work is done will require matching those hardcoded values to your robot.

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

---


## Special Case: Adding a Second Gripper (Dual Gripper)

This section covers what changes if you want a second robot that uses the same
**gripper hardware and motion logic** as robot1.

### What is different from a plain robot3

The gripper server has several hardcoded robot1-specific names that must be
parameterised or overridden:

| Currently hardcoded in `gripper_action_server.hpp` | What it needs to become |
|---|---|
| `robot1_left_finger_joint`, `robot1_right_finger_joint` | param `gripper_joint_names` |
| `robot1_tool0`, `robot1_link_6_t`, `robot1_gripper_base` … | param `touch_links` |
| `robot1_link_6_t` (attach link) | param `attach_link` |
| `/gripper_controller/follow_joint_trajectory` | param `gripper_controller_topic` |

`gripper_controller_topic` is already a parameter in `app_gripper_server.launch.py`.
The finger joint names and touch links are currently compiled in. You would need to
either:

- **Parameterise them** — expose `gripper_joint_names` and `touch_links` as
  `std::vector<std::string>` parameters in `GripperActionServer`, then read them
  in `on_configure` instead of using the hardcoded defaults.
- **Subclass or duplicate** — create `gripper_action_server2` that replicates the
  logic with robot2-prefixed names.

The parameterised approach is recommended.

### Additional URDF changes

You need a second gripper hardware block. In `dual_robot.xacro`:

```xml
<!-- Robot 3 is robot1-style: gp25 arm + gripper end effector -->
<xacro:gp25_arm_macro parent="world" prefix="robot3_" x="..." y="..." .../>
<xacro:gripper_macro  prefix="robot3_"/>
<joint name="robot3_tool0_to_gripper" type="fixed">
  <parent link="robot3_tool0"/>
  <child  link="robot3_gripper_base"/>
  <origin xyz="0 0 0.025" rpy="3.14159 0 0"/>
</joint>
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

In `app_robot3_gripper_server.launch.py`:

```python
parameters=[{
    'arm_group_name':          'robot3_arm',
    'gripper_controller_topic': '/robot3_gripper_controller/joint_trajectory',
    'gripper_joint_names':     ['robot3_left_finger_joint', 'robot3_right_finger_joint'],
    'touch_links':             ['robot3_tool0', 'robot3_link_6_t', 'robot3_flange',
                                'robot3_gripper_base', 'robot3_left_finger', 'robot3_right_finger'],
    'attach_link':             'robot3_link_6_t',
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
```
