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
Spawn collision objects in Gazebo and MoveIt planning scene.

Spawns both physical objects in Gazebo simulation and collision objects
in MoveIt planning scene for motion planning.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def quaternion_to_euler(qx, qy, qz, qw):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def generate_launch_description():
    """Spawn collision objects."""
    bringup_pkg = get_package_share_directory('hold_and_weld_bringup')

    declared_arguments = [
        DeclareLaunchArgument(
            'spawn_in_gazebo',
            default_value='true',
            description='Spawn objects in Gazebo simulation',
        ),
        DeclareLaunchArgument(
            'add_to_planning_scene',
            default_value='true',
            description='Add objects to MoveIt planning scene',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),
    ]

    spawn_in_gazebo = LaunchConfiguration('spawn_in_gazebo')
    add_to_planning_scene = LaunchConfiguration('add_to_planning_scene')
    use_sim_time = LaunchConfiguration('use_sim_time')

    objects_yaml_path = os.path.join(
        bringup_pkg, 'config', 'objects', 'objects.yaml'
    )

    with open(objects_yaml_path, 'r') as file:
        objects_yaml_dict = yaml.safe_load(file)
    objects_config = objects_yaml_dict.get('/**', {}).get('ros__parameters', {})

    child_link_config = objects_config.get('child_link', {})
    base_link_config = objects_config.get('base_link', {})

    child_link_urdf_path = child_link_config.get('urdf_path', '')
    child_link_spawn_name = child_link_config.get('spawn_name', 'child_link')
    child_link_pose = child_link_config.get('pose', {})

    base_link_urdf_path = base_link_config.get('urdf_path', '')
    base_link_spawn_name = base_link_config.get('spawn_name', 'base_link')
    base_link_pose = base_link_config.get('pose', {})

    child_link_xacro_file = PathJoinSubstitution([
        FindPackageShare('hold_and_weld_description'),
        child_link_urdf_path,
    ])
    child_link_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            child_link_xacro_file,
        ]),
        value_type=str,
    )

    base_link_xacro_file = PathJoinSubstitution([
        FindPackageShare('hold_and_weld_description'),
        base_link_urdf_path,
    ])
    base_link_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            base_link_xacro_file,
        ]),
        value_type=str,
    )

    child_roll, child_pitch, child_yaw = quaternion_to_euler(
        child_link_pose.get('qx', 0.0),
        child_link_pose.get('qy', 0.0),
        child_link_pose.get('qz', 0.0),
        child_link_pose.get('qw', 1.0)
    )

    base_roll, base_pitch, base_yaw = quaternion_to_euler(
        base_link_pose.get('qx', 0.0),
        base_link_pose.get('qy', 0.0),
        base_link_pose.get('qz', 0.0),
        base_link_pose.get('qw', 1.0)
    )

    spawn_child_link = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_child_link',
        arguments=[
            '-string', child_link_description_content.value,
            '-name', child_link_spawn_name,
            '-x', str(child_link_pose.get('x', 1.2)),
            '-y', str(child_link_pose.get('y', 0.3)),
            '-z', str(child_link_pose.get('z', 0.125)),
            '-R', str(child_roll),
            '-P', str(child_pitch),
            '-Y', str(child_yaw),
        ],
        output='screen',
        condition=IfCondition(spawn_in_gazebo),
    )

    spawn_base_link = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_base_link',
        arguments=[
            '-string', base_link_description_content.value,
            '-name', base_link_spawn_name,
            '-x', str(base_link_pose.get('x', 1.2)),
            '-y', str(base_link_pose.get('y', -0.5)),
            '-z', str(base_link_pose.get('z', 0.65)),
            '-R', str(base_roll),
            '-P', str(base_pitch),
            '-Y', str(base_yaw),
            '-static',
        ],
        output='screen',
        condition=IfCondition(spawn_in_gazebo),
    )

    child_link_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='child_link_tf_bridge',
        arguments=[
            ['/model/', child_link_spawn_name, '/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        ],
        remappings=[
            (['/model/', child_link_spawn_name, '/pose'], '/tf'),
        ],
        output='screen',
        condition=IfCondition(spawn_in_gazebo),
    )

    base_link_tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='base_link_tf_bridge',
        arguments=[
            ['/model/', base_link_spawn_name, '/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        ],
        remappings=[
            (['/model/', base_link_spawn_name, '/pose'], '/tf'),
        ],
        output='screen',
        condition=IfCondition(spawn_in_gazebo),
    )

    add_collision_objects = Node(
        package='hold_and_weld_application',
        executable='add_collision_objects.py',
        name='add_collision_objects',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(add_to_planning_scene),
    )

    nodes = [
        spawn_child_link,
        spawn_base_link,
        child_link_tf_bridge,
        base_link_tf_bridge,
        add_collision_objects,
    ]

    return LaunchDescription(declared_arguments + nodes)
