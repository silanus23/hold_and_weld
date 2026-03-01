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
MoveIt move_group launch file.

Launches the MoveIt move_group node with planning and execution capabilities.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    """Load a YAML file from a package."""
    package_share = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_share, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError as e:
        print(f'Error loading {absolute_file_path}: {e}')
        return {}


def generate_launch_description():
    """Launch MoveIt move_group node."""
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'robot_description_file',
            default_value='dual_robot.srdf',
            description='SRDF file for semantic robot description',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='WARN',
            description='Log level for move_group node',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    srdf_file = os.path.join(desc_pkg, 'config', 'dual_robot.srdf')
    with open(srdf_file, 'r') as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    kinematics_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/kinematics.yaml'
    )
    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )
    }

    joint_limits_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/joint_limits.yaml'
    )
    joint_limits_config = joint_limits_yaml_dict.get('/**', {}).get(
        'ros__parameters', {}
    )

    ompl_planning_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/ompl_planning.yaml'
    )
    ompl_planning_config = ompl_planning_yaml_dict.get('/**', {}).get(
        'ros__parameters', {}
    )

    moveit_controllers_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/moveit_controllers.yaml'
    )
    moveit_controllers_config = moveit_controllers_yaml_dict.get('/**', {}).get(
        'ros__parameters', {}
    )

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    planning_pipeline_config = {
        'move_group': {'planning_plugins': ['ompl_interface/OMPLPlanner']}
    }

    # robot_description comes from /robot_description topic published by robot_state_publisher
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {
                'robot_description_topic': '/robot_description'
            },
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            trajectory_execution,
            {
                'moveit_simple_controller_manager':
                    moveit_controllers_config.get(
                        'moveit_simple_controller_manager', {}
                    )
            },
            {
                'moveit_controller_manager': moveit_controllers_config.get(
                    'moveit_controller_manager',
                    'moveit_simple_controller_manager/'
                    'MoveItSimpleControllerManager',
                )
            },
            planning_scene_monitor_parameters,
            planning_pipeline_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group])
