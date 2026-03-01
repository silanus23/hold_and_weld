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
Controller spawner launch file.

Spawns robot controllers in sequence using event-based actions.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Spawn controllers in sequence."""
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_type',
            default_value='dual',
            description='Robot type: gripper, welder, or dual',
            choices=['gripper', 'welder', 'dual'],
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'controller_manager_timeout',
            default_value='30',
            description='Timeout for controller manager (seconds)',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    robot1_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robot1_arm_controller',
        arguments=[
            'robot1_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    robot1_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot1_arm_controller_spawner],
        )
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_gripper_controller',
        arguments=[
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot1_arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    robot2_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robot2_arm_controller',
        arguments=[
            'robot2_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    robot2_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[robot2_arm_controller_spawner],
        )
    )

    nodes = [
        joint_state_broadcaster,
        robot1_arm_controller,
        gripper_controller,
        robot2_arm_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
