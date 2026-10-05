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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Controllers each robot_type spawns after joint_state_broadcaster, in order. A
# controller missing from the controller_manager fails its spawner, so each bringup
# must only name the slots its URDF has.
CONTROLLERS_BY_ROBOT_TYPE = {
    'gripper': ['robot1_arm_controller', 'robot1_gripper_controller'],
    'welder': ['robot2_arm_controller'],
    'dual': ['robot1_arm_controller', 'robot1_gripper_controller', 'robot2_arm_controller'],
}


def spawner(controller, use_sim_time, controller_manager_timeout):
    """Spawner node for one controller."""
    return Node(
        package='controller_manager',
        executable='spawner',
        name=f'spawner_{controller}',
        arguments=[
            controller,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', controller_manager_timeout,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )


def launch_setup(context):
    """Chain the spawners of the selected robot_type, each after the previous exits."""
    robot_type = LaunchConfiguration('robot_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager_timeout = LaunchConfiguration('controller_manager_timeout')

    previous = spawner('joint_state_broadcaster', use_sim_time, controller_manager_timeout)
    actions = [previous]
    for controller in CONTROLLERS_BY_ROBOT_TYPE[robot_type]:
        current = spawner(controller, use_sim_time, controller_manager_timeout)
        actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(target_action=previous, on_exit=[current])
        ))
        previous = current
    return actions


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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
