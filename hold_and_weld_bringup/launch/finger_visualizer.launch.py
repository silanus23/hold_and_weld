# Copyright 2026 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Grasp visualizer launch file.

Counterpart to magic_wand.launch.py for the holding gripper side.

Differences from magic_wand.launch.py:
  - Launches finger_visualizer.py instead of magic_wand.py.
  - finger_visualizer.py spawns objects at their START pose (where the gripper
    picks them up) rather than the welded end pose.
  - Visualizes gripper finger boxes from the latest grasps/*.json.
  - Does NOT launch add_collision_objects.py — finger_visualizer.py handles
    spawning internally so objects appear at the correct start position.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_launch_description():
    """Launch grasp visualizer system."""
    bringup_pkg = get_package_share_directory('hold_and_weld_bringup')
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization',
        ),
        DeclareLaunchArgument(
            'publish_jaw_clearance',
            default_value='true',
            description='Publish jaw-clearance cylinders on /jaw_clearance_markers',
        ),
        DeclareLaunchArgument(
            'publish_constraints',
            default_value='true',
            description='Publish exclusion-zone/secondary geometry on /constraint_markers',
        ),
        DeclareLaunchArgument(
            'max_grasps',
            default_value='0',
            description='Visualize only the first N grasps from the JSON (0 = all)',
        ),
    ]

    use_rviz = LaunchConfiguration('use_rviz')
    publish_jaw_clearance = LaunchConfiguration('publish_jaw_clearance')
    publish_constraints = LaunchConfiguration('publish_constraints')
    max_grasps = LaunchConfiguration('max_grasps')

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', 'finger_vis.rviz']
    )

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('hold_and_weld_description'),
                'urdf',
                'robot1_gripper.xacro',
            ]),
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    srdf_file = os.path.join(desc_pkg, 'config', 'robot1_gripper.srdf')
    with open(srdf_file, 'r') as fh:
        robot_description_semantic_content = fh.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    kinematics_yaml_path = os.path.join(
        bringup_pkg, 'config', 'moveit', 'kinematics.yaml'
    )
    with open(kinematics_yaml_path, 'r') as fh:
        kinematics_cfg = yaml.safe_load(fh)
    kinematics_config = {
        'robot_description_kinematics': kinematics_cfg.get('/**', {}).get(
            'ros__parameters', {}
        )
    }

    joint_limits_yaml_path = os.path.join(
        bringup_pkg, 'config', 'moveit', 'joint_limits.yaml'
    )
    with open(joint_limits_yaml_path, 'r') as fh:
        joint_limits_cfg = yaml.safe_load(fh)
    joint_limits_config = joint_limits_cfg.get('/**', {}).get(
        'ros__parameters', {}
    )

    ompl_planning_yaml_path = os.path.join(
        bringup_pkg, 'config', 'moveit', 'ompl_planning.yaml'
    )
    with open(ompl_planning_yaml_path, 'r') as fh:
        ompl_planning_cfg = yaml.safe_load(fh)
    ompl_planning_config = ompl_planning_cfg.get('/**', {}).get(
        'ros__parameters', {}
    )

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ompl_planning.yaml is pipeline-scoped, so it nests under the 'ompl' id.
    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_config,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': False}, robot_description],
    )

    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': False}],
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planning_scene_monitor_parameters,
            planning_pipeline_config,
            {'use_sim_time': False},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            {'use_sim_time': False},
        ],
        condition=IfCondition(use_rviz),
    )

    finger_visualizer = Node(
        package='hold_and_weld_application',
        executable='finger_visualizer.py',
        name='finger_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'publish_jaw_clearance': publish_jaw_clearance,
            'publish_constraints': publish_constraints,
            'max_grasps': max_grasps,
        }],
    )

    delay_move_group = TimerAction(period=2.0, actions=[move_group])
    delay_rviz = TimerAction(period=5.0, actions=[rviz])
    delay_finger_visualizer = TimerAction(period=9.0, actions=[finger_visualizer])

    nodes = [
        robot_state_publisher,
        static_tf_world,
        delay_move_group,
        delay_rviz,
        delay_finger_visualizer,
    ]

    return LaunchDescription(declared_arguments + nodes)
