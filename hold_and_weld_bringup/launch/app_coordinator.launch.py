# Copyright 2025 Berkan Tali
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
Dual robot coordinator lifecycle launch file.

Launches the coordinator as a lifecycle node with event-based
state management for orchestrating gripper and welder operations.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml


def load_yaml(package_name, file_path):
    """Load a YAML file from a package."""
    package_share = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_share, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    """Launch dual robot coordinator as a lifecycle node."""
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Automatically start coordinated sequence when system is ready',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_start = LaunchConfiguration('auto_start')

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

    coordinator = LifecycleNode(
        package='hold_and_weld_application',
        executable='dual_robot_coordinator',
        name='dual_robot_coordinator',
        namespace='',
        output='screen',
        parameters=[
            robot_description_semantic,
            kinematics_config,
            {'use_sim_time': use_sim_time},
            {'auto_start': auto_start},
        ],
    )

    coordinator_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=coordinator,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(coordinator),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
    )

    coordinator_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=coordinator,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(coordinator),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )

    return LaunchDescription(
        declared_arguments + [
            coordinator,
            coordinator_configure_event_handler,
            coordinator_activate_event_handler,
        ]
    )
