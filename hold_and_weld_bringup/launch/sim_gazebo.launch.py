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
Gazebo simulation launch file.

Launches Gazebo server, ROS-Gazebo bridge, and robot state publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch Gazebo simulation with robot."""
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_paths = [desc_pkg, os.path.dirname(desc_pkg)]
    if gz_resource_path:
        resource_paths.append(gz_resource_path)
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(resource_paths)

    declared_arguments = [
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Launch Gazebo with GUI (set to false for headless mode)',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='bringup_world.world',
            description='Gazebo world file name',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='dual_gp25_system',
            description='Name of the robot in Gazebo',
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='dual_robot.xacro',
            description='URDF/xacro file for the robot',
        ),
        DeclareLaunchArgument(
            'controller_config',
            default_value='controllers.yaml',
            description='Controller configuration file',
        ),
    ]

    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')
    urdf_file = LaunchConfiguration('urdf_file')
    controller_config = LaunchConfiguration('controller_config')

    world_path = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'worlds', world_file]
    )

    controller_config_path = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'config', controller_config]
    )

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('hold_and_weld_description'),
                'urdf',
                urdf_file,
            ]),
            ' ',
            'controller_config_file:=',
            controller_config_path,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    gz_args = PythonExpression([
        "'", '-r -v 4 ', "' if '", use_gazebo_gui, "' == 'true' else '", '-r -v 4 -s ', "'"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments=[
            ('gz_args', [gz_args, ' ', world_path]),
            ('on_exit_shutdown', 'true'),
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ['/world/default/model/', robot_name, '/joint_state',
             '@sensor_msgs/msg/JointState[gz.msgs.Model'],
        ],
        remappings=[
            ('/world/default/clock', '/clock'),
            (['/world/default/model/', robot_name, '/joint_state'], '/joint_states'),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=['spawn_', robot_name],
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    nodes = [
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_robot,
    ]

    return LaunchDescription(declared_arguments + nodes)
