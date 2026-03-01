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
RViz launch file with MoveIt motion planning plugin.

Launches RViz2 with MoveIt configuration for visualizing robot motion planning.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    """Launch RViz with MoveIt plugin."""
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    declared_arguments = [
        DeclareLaunchArgument(
            'rviz_config',
            default_value='moveit.rviz',
            description='RViz config file name',
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

    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', rviz_config]
    )

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

    joint_limits_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/joint_limits.yaml'
    )
    joint_limits_config = joint_limits_yaml_dict.get('/**', {}).get(
        'ros__parameters', {}
    )

    ompl_planning_yaml_dict = load_yaml(
        'hold_and_weld_bringup', 'config/moveit/ompl_planning.yaml'
    )
    ompl_planning_config = ompl_planning_yaml_dict.get('/**', {}).get(
        'ros__parameters', {}
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(declared_arguments + [rviz])
