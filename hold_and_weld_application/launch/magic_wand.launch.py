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
Magic wand launch file.

Launches visualization system with MoveIt, collision objects,
and interactive torch tip markers from trajectory JSON.
"""

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    """Launch magic wand visualization system."""
    # PACKAGE DIRECTORIES
    app_pkg = get_package_share_directory('hold_and_weld_application')
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    # LAUNCH ARGUMENTS
    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization',
        ),
    ]

    use_rviz = LaunchConfiguration('use_rviz')

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', 'magic_wand.rviz']
    )

    # ROBOT DESCRIPTION (URDF) - Robot 1 only
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

    # ROBOT SEMANTIC DESCRIPTION (SRDF)
    srdf_file = os.path.join(desc_pkg, 'config', 'robot1_gripper.srdf')
    with open(srdf_file, 'r') as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # MOVEIT CONFIGURATION
    kinematics_yaml_path = os.path.join(app_pkg, 'config', 'system', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml_dict = yaml.safe_load(file)
    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )
    }

    joint_limits_yaml_path = os.path.join(app_pkg, 'config', 'system', 'joint_limits.yaml')
    with open(joint_limits_yaml_path, 'r') as file:
        joint_limits_yaml_dict = yaml.safe_load(file)
        joint_limits_config = joint_limits_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    ompl_planning_yaml_path = os.path.join(app_pkg, 'config', 'system', 'ompl_planning.yaml')
    with open(ompl_planning_yaml_path, 'r') as file:
        ompl_planning_yaml_dict = yaml.safe_load(file)
        ompl_planning_config = ompl_planning_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    # MoveIt configuration parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    planning_pipeline_config = {
        'move_group': {'planning_plugins': ['ompl_interface/OMPLPlanner']}
    }

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': False}, robot_description],
    )

    # STATIC TRANSFORM PUBLISHER (world frame)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': False}],
    )

    # MOVEIT MOVE_GROUP
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
            ompl_planning_config,
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

    # COLLISION OBJECTS
    add_collision_objects = Node(
        package='hold_and_weld_application',
        executable='add_collision_objects.py',
        name='add_collision_objects',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    magic_wand = Node(
        package='hold_and_weld_application',
        executable='magic_wand.py',
        name='magic_wand',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    delay_move_group = TimerAction(period=2.0, actions=[move_group])
    delay_rviz = TimerAction(period=5.0, actions=[rviz])
    delay_add_collision_objects = TimerAction(period=7.0, actions=[add_collision_objects])
    delay_magic_wand = TimerAction(period=9.0, actions=[magic_wand])

    nodes = [
        robot_state_publisher,
        static_tf_world,
        delay_move_group,
        delay_rviz,
        delay_add_collision_objects,
        delay_magic_wand,
    ]

    return LaunchDescription(declared_arguments + nodes)
