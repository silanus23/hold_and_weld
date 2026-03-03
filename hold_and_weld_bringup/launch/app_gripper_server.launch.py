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
Gripper action server lifecycle launch file.

Launches the gripper action server as a lifecycle node with event-based
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
    """Launch gripper action server as a lifecycle node."""
    bringup_pkg = get_package_share_directory('hold_and_weld_bringup')
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'arm_group_name',
            default_value='robot1_gp25_arm',
            description='MoveIt planning group name for gripper arm',
        ),
        DeclareLaunchArgument(
            'positions_yaml',
            default_value='pick_place_targets.yaml',
            description='Pick and place targets configuration file',
        ),
        DeclareLaunchArgument(
            'auto_trigger',
            default_value='false',
            description='Automatically trigger gripper job on startup',
        ),
        DeclareLaunchArgument(
            'auto_trigger_delay_sec',
            default_value='5.0',
            description='Delay before auto-triggering gripper job (seconds)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
    ]

    arm_group_name = LaunchConfiguration('arm_group_name')
    auto_trigger = LaunchConfiguration('auto_trigger')
    auto_trigger_delay_sec = LaunchConfiguration('auto_trigger_delay_sec')
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

    positions_yaml_path = os.path.join(
        bringup_pkg, 'config', 'tasks', 'pick_place_targets.yaml'
    )

    gripper_server = LifecycleNode(
        package='hold_and_weld_application',
        executable='gripper_server',
        name='gripper_action_server',
        namespace='',
        output='screen',
        parameters=[
            robot_description_semantic,
            kinematics_config,
            {
                'arm_group_name': arm_group_name,
                'positions_yaml': positions_yaml_path,
                'gripper_controller_topic': '/gripper_controller/joint_trajectory',
                'auto_trigger': auto_trigger,
                'auto_trigger_delay_sec': auto_trigger_delay_sec,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    gripper_server_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gripper_server,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(gripper_server),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
    )

    gripper_server_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=gripper_server,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(gripper_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )

    return LaunchDescription(
        declared_arguments + [
            gripper_server,
            gripper_server_configure_event_handler,
            gripper_server_activate_event_handler,
        ]
    )
