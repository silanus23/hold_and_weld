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
Welder action server lifecycle launch file.

Launches the welder action server as a lifecycle node with event-based
state management.
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
    """Launch welder action server as a lifecycle node."""
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'welder_group_name',
            default_value='robot2_welder_arm',
            description='MoveIt planning group name for welder arm',
        ),
        DeclareLaunchArgument(
            'auto_load_latest',
            default_value='true',
            description='Automatically load latest trajectory JSON file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
    ]

    welder_group_name = LaunchConfiguration('welder_group_name')
    auto_load_latest = LaunchConfiguration('auto_load_latest')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    welder_server = LifecycleNode(
        package='hold_and_weld_application',
        executable='welder_server',
        name='welder_action_server',
        namespace='',
        output='screen',
        parameters=[
            robot_description_semantic,
            kinematics_config,
            {
                'welder_group_name': welder_group_name,
                'auto_load_latest': auto_load_latest,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    welder_server_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=welder_server,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(welder_server),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
    )

    welder_server_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=welder_server,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(welder_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )

    return LaunchDescription(
        declared_arguments + [
            welder_server,
            welder_server_configure_event_handler,
            welder_server_activate_event_handler,
        ]
    )
