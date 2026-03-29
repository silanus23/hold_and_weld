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
in parallel.  Each node waits for its own dependencies internally, matching
the approach used in system_bringup.launch.py.

Robot2 is spawned at the safety pose defined in config/tasks/welding.yaml so
that the workcell starts in a collision-free configuration without requiring
a MoveIt safety move at runtime.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    """Launch welder-only system with parallel node startup."""
    # Read robot2 safety joint positions from welding.yaml and flatten into
    # [-J name value ...] spawn arguments for Gazebo.
    welding_yaml = load_yaml('hold_and_weld_bringup', 'config/tasks/welding.yaml')
    safety_joints = (
        welding_yaml.get('safety_pose', {}).get('joint_positions', {})
        if welding_yaml else {}
    )
    spawn_joint_args = [arg for jn, jv in safety_joints.items() for arg in ['-J', jn, str(jv)]]

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
    ]

    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_bringup'), 'launch']
    )

    # Gazebo — internal spawn disabled; we handle it below with -J args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_gazebo.launch.py']),
        launch_arguments={
            'use_gazebo_gui': use_gazebo_gui,
            'robot_name': 'gp25_welder_system',
            'urdf_file': 'robot2_welder.xacro',
            'controller_config': 'robot2_controllers.yaml',
            'spawn_robot': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Spawn robot2 at the safety pose from welding.yaml
    spawn_welder = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gp25_welder_system',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'gp25_welder_system',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0',
        ] + spawn_joint_args,
        output='screen',
    )

    # Collision objects — spawned in Gazebo and added to planning scene
    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'true',
            'add_to_planning_scene': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Controllers — wait for controller_manager internally
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/controllers_spawn.launch.py']),
        launch_arguments={
            'robot_type': 'welder',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # MoveIt move_group — waits for robot_description and joint_states internally
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_move_group.launch.py']),
        launch_arguments={
            'robot_description_file': 'robot2_welder.srdf',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_rviz.launch.py']),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Add collision objects to planning scene
    add_collision_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'false',
            'add_to_planning_scene': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Welder application server — lifecycle node, waits for its dependencies
    welder_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/app_welder_server.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nodes = [
        gazebo,
        spawn_welder,
        spawn_objects,
        controllers,
        move_group,
        rviz,
        add_collision_objects,
        welder_server,
    ]

    return LaunchDescription(declared_arguments + nodes)
