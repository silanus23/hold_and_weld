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
Welder-only system bringup.

Launches complete welder system: Gazebo, MoveIt, RViz, and welder application
using event-based sequencing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch welder-only system with event-based sequencing."""
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
    ]

    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')

    bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_bringup'), 'launch']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_gazebo.launch.py']),
        launch_arguments={
            'use_gazebo_gui': use_gazebo_gui,
            'robot_name': 'gp25_welder_system',
            'urdf_file': 'robot2_welder.xacro',
            'controller_config': 'robot2_controllers.yaml',
        }.items(),
    )

    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'true',
            'add_to_planning_scene': 'true',
        }.items(),
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/controllers_spawn.launch.py']),
        launch_arguments={
            'robot_type': 'welder',
        }.items(),
    )

    controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[controllers_launch],
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_move_group.launch.py']),
        launch_arguments={
            'robot_description_file': 'robot2_welder.srdf',
        }.items(),
    )

    move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controllers_launch,
            on_exit=[move_group_launch],
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_rviz.launch.py']),
        launch_arguments={
            'use_rviz': use_rviz,
        }.items(),
    )

    rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=move_group_launch,
            on_exit=[rviz_launch],
        )
    )

    add_collision_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'false',
            'add_to_planning_scene': 'true',
        }.items(),
    )

    add_collision_objects = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_launch,
            on_exit=[add_collision_objects_launch],
        )
    )

    welder_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/app_welder_server.launch.py']),
    )

    welder_server = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=add_collision_objects_launch,
            on_exit=[welder_server_launch],
        )
    )

    nodes = [
        gazebo,
        spawn_objects,
        controllers,
        move_group,
        rviz,
        add_collision_objects,
        welder_server,
    ]

    return LaunchDescription(declared_arguments + nodes)
