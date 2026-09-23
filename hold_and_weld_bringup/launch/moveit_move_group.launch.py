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
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from launch_utils import load_yaml  # noqa: E402, I100

PIPELINE_YAML_PATHS = {
    'ompl': 'config/moveit/ompl_planning.yaml',
    'pilz_industrial_motion_planner': 'config/moveit/pilz_industrial_motion_planner_planning.yaml',
}


def launch_setup(context, *args, **kwargs):
    """Build the move_group node.

    Run via OpaqueFunction: which pipeline YAMLs to load is only known once
    'planning_pipelines' is resolved at launch time, not at launch-file-generation
    time, so this can't be plain top-level generate_launch_description() code.
    """
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    pipeline_names = [
        name.strip() for name in
        LaunchConfiguration('planning_pipelines').perform(context).split(',')
        if name.strip()
    ]
    default_pipeline = LaunchConfiguration('default_planning_pipeline').perform(context)

    unknown_pipelines = [name for name in pipeline_names if name not in PIPELINE_YAML_PATHS]
    if unknown_pipelines:
        raise ValueError(
            f'Unknown planning pipeline(s) {unknown_pipelines}: no entry in '
            f'PIPELINE_YAML_PATHS (known: {list(PIPELINE_YAML_PATHS)})'
        )
    if default_pipeline not in pipeline_names:
        raise ValueError(
            f"default_planning_pipeline '{default_pipeline}' must be one of "
            f'planning_pipelines {pipeline_names}'
        )

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

    cartesian_limits_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/cartesian_limits.yaml'
    )
    cartesian_limits_config = cartesian_limits_yaml_dict.get('/**', {}).get(
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

    # Root-level layout (as moveit_configs_builder emits it): move_group reads
    # 'planning_pipelines' / 'default_planning_pipeline' and '<name>.*' from its
    # own root.
    planning_pipeline_config = {
        'planning_pipelines': pipeline_names,
        'default_planning_pipeline': default_pipeline,
    }
    for pipeline_name in pipeline_names:
        pipeline_yaml_dict = load_yaml(
            'hold_and_weld_bringup', PIPELINE_YAML_PATHS[pipeline_name]
        )
        planning_pipeline_config[pipeline_name] = pipeline_yaml_dict.get(
            '/**', {}
        ).get('ros__parameters', {})

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
            cartesian_limits_config,
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

    return [move_group]


def generate_launch_description():
    """Launch MoveIt move_group node."""
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
        DeclareLaunchArgument(
            'planning_pipelines',
            default_value='ompl,pilz_industrial_motion_planner',
            description=(
                'Comma-separated list of planning pipeline ids to load into '
                f'move_group (known ids: {list(PIPELINE_YAML_PATHS)})'
            ),
        ),
        DeclareLaunchArgument(
            'default_planning_pipeline',
            default_value='ompl',
            description=(
                'Pipeline id used when a planning request does not specify one; '
                'must be a member of planning_pipelines'
            ),
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
