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
Gripper-only system bringup.

Launches complete gripper system: Gazebo, MoveIt, RViz, and gripper application
in parallel, matching system_bringup.launch.py. The action server and controller
spawners wait for their dependencies; add_collision_objects.py does not.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch gripper-only system with parallel node startup."""
    declared_arguments = [
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Launch Gazebo with GUI',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'auto_trigger',
            default_value='true',
            description='Auto-trigger gripper job after startup',
        ),
        DeclareLaunchArgument(
            'auto_trigger_delay_sec',
            default_value='5.0',
            description='Delay before auto-trigger (seconds)',
        ),
        DeclareLaunchArgument(
            'move_group_log_level',
            default_value='WARN',
            description='move_group log level (e.g. DEBUG for OMPL/collision detail)',
        ),
    ]

    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_trigger = LaunchConfiguration('auto_trigger')
    auto_trigger_delay_sec = LaunchConfiguration('auto_trigger_delay_sec')
    move_group_log_level = LaunchConfiguration('move_group_log_level')

    bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_bringup'), 'launch']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_gazebo.launch.py']),
        launch_arguments={
            'use_gazebo_gui': use_gazebo_gui,
            'robot_name': 'gripper_system',
            'urdf_file': 'robot1_gripper.xacro',
            'controller_config': 'robot1_controllers.yaml',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'true',
            'add_to_planning_scene': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/controllers_spawn.launch.py']),
        launch_arguments={
            'robot_type': 'gripper',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_move_group.launch.py']),
        launch_arguments={
            'robot_description_file': 'robot1_gripper.srdf',
            'use_sim_time': use_sim_time,
            'log_level': move_group_log_level,
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_rviz.launch.py']),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    add_collision_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'false',
            'add_to_planning_scene': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    gripper_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/app_gripper_server.launch.py']),
        launch_arguments={
            'auto_trigger': auto_trigger,
            'auto_trigger_delay_sec': auto_trigger_delay_sec,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nodes = [
        gazebo,
        spawn_objects,
        controllers,
        move_group,
        rviz,
        add_collision_objects,
        gripper_server,
    ]

    return LaunchDescription(declared_arguments + nodes)
