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
Workcell configurator launch file.

Starts the configurator node and RViz2 with its palette, the 2D Goal Pose tool
retargeted to placement, and the Measure tool. Standalone: nothing else needs to
run. workcell_file defaults to the pinned editing template
hold_and_weld_description/config/workcell.configurator.yaml, not the live
workcell.yaml, so every launch starts from the same fixed layout regardless of
what a previous session saved. Saving writes to staging files in
hold_and_weld_bringup/config/configurator_output (git-ignored); copy them
into the live configs yourself to apply a layout.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the workcell configurator."""
    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization',
        ),
        DeclareLaunchArgument(
            'workcell_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hold_and_weld_description'), 'config',
                 'workcell.configurator.yaml']),
            description=(
                'Robot slot layout to load every launch (pinned editing template, not '
                'the live workcell.yaml)'),
        ),
        DeclareLaunchArgument(
            'objects_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('hold_and_weld_bringup'), 'config', 'objects', 'objects.yaml']),
            description='Object slot layout to load and save',
        ),
        DeclareLaunchArgument(
            'workcell_output_file',
            default_value='',
            description=(
                'Staging file robot slots are saved to (default: '
                'config/configurator_output/workcell.yaml); copy it into workcell_file '
                'yourself to apply it'),
        ),
        DeclareLaunchArgument(
            'objects_output_file',
            default_value='',
            description=(
                'Staging file object slots are saved to (default: '
                'config/configurator_output/objects.yaml); copy it into objects_file '
                'yourself to apply it'),
        ),
    ]

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', 'workcell_configurator.rviz']
    )

    # RViz2's fixed frame must exist in TF even when no robot is running.
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='workcell_configurator_frame_publisher',
        arguments=['--frame-id', 'world', '--child-frame-id', 'workcell_configurator'],
    )

    configurator = Node(
        package='hold_and_weld_bringup',
        executable='workcell_configurator.py',
        name='workcell_configurator',
        output='screen',
        parameters=[{
            'workcell_file': LaunchConfiguration('workcell_file'),
            'objects_file': LaunchConfiguration('objects_file'),
            'workcell_output_file': LaunchConfiguration('workcell_output_file'),
            'objects_output_file': LaunchConfiguration('objects_output_file'),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(declared_arguments + [static_tf_world, configurator, rviz])
