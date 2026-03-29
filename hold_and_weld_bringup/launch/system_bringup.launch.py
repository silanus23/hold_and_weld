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
Complete dual-robot system bringup.

Launches full system: Gazebo, MoveIt, RViz, gripper, welder, and coordinator.
Uses industry-standard parallel launch approach where nodes handle their own dependencies.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    """Launch complete dual-robot welding system."""
    # Read robot2 safety joint positions from welding.yaml and flatten into
    # [-J name value ...] spawn arguments for Gazebo.
    welding_yaml = load_yaml('hold_and_weld_bringup', 'config/tasks/welding.yaml')
    safety_joints = (
        welding_yaml.get('safety_pose', {}).get('joint_positions', {})
        if welding_yaml else {}
    )
    # Build xacro args string for robot2 initial joint positions.
    # Joint order in gp25_arm_prefix.xacro: j1=joint_1_s, j2=joint_2_l, ...
    joint_index_map = {
        'robot2_joint_1_s': 'robot2_initial_pos_j1',
        'robot2_joint_2_l': 'robot2_initial_pos_j2',
        'robot2_joint_3_u': 'robot2_initial_pos_j3',
        'robot2_joint_4_r': 'robot2_initial_pos_j4',
        'robot2_joint_5_b': 'robot2_initial_pos_j5',
        'robot2_joint_6_t': 'robot2_initial_pos_j6',
    }
    robot2_initial_positions = ' '.join(
        f'{xacro_arg}:={safety_joints[joint_name]}'
        for joint_name, xacro_arg in joint_index_map.items()
        if joint_name in safety_joints
    )

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
            'auto_start',
            default_value='true',
            description='Automatically start coordinated sequence when system is ready',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
    ]

    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_bringup'), 'launch']
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_gazebo.launch.py']),
        launch_arguments={
            'use_gazebo_gui': use_gazebo_gui,
            'robot_name': 'dual_gp25_system',
            'urdf_file': 'dual_robot.xacro',
            'controller_config': 'controllers.yaml',
            'robot2_initial_positions': robot2_initial_positions,
        }.items(),
    )

    # Spawn objects in Gazebo (parallel with Gazebo startup)
    spawn_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'true',
            'add_to_planning_scene': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Controllers (will wait for controller_manager via internal timeout)
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/controllers_spawn.launch.py']),
        launch_arguments={
            'robot_type': 'dual',
            'controller_manager_timeout': '30',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # MoveIt move_group (will wait for robot_description and joint_states internally)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_move_group.launch.py']),
        launch_arguments={
            'robot_description_file': 'dual_robot.srdf',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # RViz (will connect when move_group is available)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/moveit_rviz.launch.py']),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Add collision objects to planning scene (will wait for planning_scene service)
    add_collision_objects = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/sim_spawn_objects.launch.py']),
        launch_arguments={
            'spawn_in_gazebo': 'false',
            'add_to_planning_scene': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Application servers (lifecycle nodes that will wait for their dependencies)
    welder_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/app_welder_server.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    gripper_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/app_gripper_server.launch.py']),
        launch_arguments={
            'auto_trigger': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Coordinator (lifecycle node that orchestrates the system)
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_launch_dir, '/app_coordinator.launch.py']),
        launch_arguments={
            'auto_start': auto_start,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Launch everything in parallel - nodes will handle their own dependencies
    nodes = [
        gazebo,
        spawn_objects,
        controllers,
        move_group,
        rviz,
        add_collision_objects,
        welder_server,
        gripper_server,
        coordinator,
    ]

    return LaunchDescription(declared_arguments + nodes)
