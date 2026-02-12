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
Magic wand launch file.

Launches visualization system with MoveIt, collision objects,
and interactive torch tip markers from trajectory JSON.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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


def generate_launch_description():
    """Launch magic wand visualization system."""
    # PACKAGE DIRECTORIES
    app_pkg = get_package_share_directory('hold_and_weld_application')
    desc_pkg = get_package_share_directory('hold_and_weld_description')

    # LAUNCH ARGUMENTS
    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization',
        ),
    ]

    use_rviz = LaunchConfiguration('use_rviz')

    # FILE PATHS
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', 'magic_wand.rviz']
    )

    # ROBOT DESCRIPTION (URDF) - Robot 1 only
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('hold_and_weld_description'),
                'urdf',
                'robot1_gripper.xacro',
            ]),
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # ROBOT SEMANTIC DESCRIPTION (SRDF)
    srdf_file = os.path.join(desc_pkg, 'config', 'robot1_gripper.srdf')
    with open(srdf_file, 'r') as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # MOVEIT CONFIGURATION
    kinematics_yaml_path = os.path.join(app_pkg, 'config', 'system', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml_dict = yaml.safe_load(file)
    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )
    }

    joint_limits_yaml_path = os.path.join(app_pkg, 'config', 'system', 'joint_limits.yaml')
    with open(joint_limits_yaml_path, 'r') as file:
        joint_limits_yaml_dict = yaml.safe_load(file)
        joint_limits_config = joint_limits_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    ompl_planning_yaml_path = os.path.join(app_pkg, 'config', 'system', 'ompl_planning.yaml')
    with open(ompl_planning_yaml_path, 'r') as file:
        ompl_planning_yaml_dict = yaml.safe_load(file)
        ompl_planning_config = ompl_planning_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    # MoveIt configuration parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    planning_pipeline_config = {
        'move_group': {'planning_plugins': ['ompl_interface/OMPLPlanner']}
    }

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': False}, robot_description],
    )

    # STATIC TRANSFORM PUBLISHER (world frame)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_frame_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        parameters=[{'use_sim_time': False}],
    )

    # MOVEIT MOVE_GROUP
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            planning_scene_monitor_parameters,
            planning_pipeline_config,
            {'use_sim_time': False},
        ],
    )

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            {'use_sim_time': False},
        ],
        condition=IfCondition(use_rviz),
    )

    # COLLISION OBJECTS
    add_collision_objects = Node(
        package='hold_and_weld_application',
        executable='add_collision_objects.py',
        name='add_collision_objects',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # MAGIC WAND (Interactive Markers)
    magic_wand = Node(
        package='hold_and_weld_application',
        executable='magic_wand.py',
        name='magic_wand',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # TIMING SEQUENCE
    delay_move_group = TimerAction(period=2.0, actions=[move_group])
    delay_rviz = TimerAction(period=5.0, actions=[rviz])
    delay_add_collision_objects = TimerAction(period=7.0, actions=[add_collision_objects])
    delay_magic_wand = TimerAction(period=9.0, actions=[magic_wand])

    # ASSEMBLE LAUNCH DESCRIPTION
    nodes = [
        robot_state_publisher,
        static_tf_world,
        delay_move_group,
        delay_rviz,
        delay_add_collision_objects,
        delay_magic_wand,
    ]

    return LaunchDescription(declared_arguments + nodes)

    # Configure Gazebo resource paths
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_paths = [desc_pkg, os.path.dirname(desc_pkg)]
    if gz_resource_path:
        resource_paths.append(gz_resource_path)
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(resource_paths)

    # LAUNCH ARGUMENTS
    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz visualization',
        ),
    ]

    use_rviz = LaunchConfiguration('use_rviz')

    # FILE PATHS
    world_path = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'worlds', 'bringup_world.world']
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', 'magic_wand.rviz']
    )

    # ROBOT DESCRIPTION (URDF)
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('hold_and_weld_description'),
                'urdf',
                'dual_robot.xacro',
            ]),
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # ROBOT SEMANTIC DESCRIPTION (SRDF)
    srdf_file = os.path.join(desc_pkg, 'config', 'dual_robot.srdf')
    with open(srdf_file, 'r') as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # MOVEIT CONFIGURATION
    kinematics_yaml_path = os.path.join(app_pkg, 'config', 'system', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as file:
        kinematics_yaml_dict = yaml.safe_load(file)
    kinematics_config = {
        'robot_description_kinematics': kinematics_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )
    }

    joint_limits_yaml_path = os.path.join(app_pkg, 'config', 'system', 'joint_limits.yaml')
    with open(joint_limits_yaml_path, 'r') as file:
        joint_limits_yaml_dict = yaml.safe_load(file)
        joint_limits_config = joint_limits_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    ompl_planning_yaml_path = os.path.join(app_pkg, 'config', 'system', 'ompl_planning.yaml')
    with open(ompl_planning_yaml_path, 'r') as file:
        ompl_planning_yaml_dict = yaml.safe_load(file)
        ompl_planning_config = ompl_planning_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    # MoveIt configuration parameters
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    planning_pipeline_config = {
        'move_group': {'planning_plugins': ['ompl_interface/OMPLPlanner']}
    }

    # LOAD OBJECT CONFIGURATION FROM YAML
    objects_yaml_path = os.path.join(app_pkg, 'config', 'collision_objects', 'objects.yaml')
    with open(objects_yaml_path, 'r') as file:
        objects_yaml_dict = yaml.safe_load(file)
    objects_config = objects_yaml_dict.get('/**', {}).get('ros__parameters', {})

    # OBJECT DESCRIPTIONS
    child_link_urdf_path = objects_config.get('child_link', {}).get('urdf_path', '')
    child_link_spawn_name = objects_config.get('child_link', {}).get('spawn_name', 'child_link')
    child_link_pose = objects_config.get('child_link', {}).get('pose', {})
    
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

    base_link_urdf_path = objects_config.get('base_link', {}).get('urdf_path', '')
    base_link_spawn_name = objects_config.get('base_link', {}).get('spawn_name', 'base_link')
    base_link_pose = objects_config.get('base_link', {}).get('pose', {})
    
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

    # GAZEBO SIMULATION
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'gz_args': ('-r -v 4 ', world_path),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ROS-GAZEBO BRIDGE
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ('/world/default/model/dual_robot/joint_state'
             '@sensor_msgs/msg/JointState[gz.msgs.Model'),
            f'/model/{child_link_spawn_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/model/{base_link_spawn_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/default/clock', '/clock'),
            ('/world/default/model/dual_robot/joint_state', '/joint_states'),
            (f'/model/{child_link_spawn_name}/pose', '/tf'),
            (f'/model/{base_link_spawn_name}/pose', '/tf'),
        ],
        output='screen',
    )

    # ROBOT STATE PUBLISHER
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    # SPAWN ENTITIES IN GAZEBO
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_dual_robot',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'dual_robot',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
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
        ],
        output='screen',
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
            '-R', str(base_link_pose.get('qx', 0.0)),
            '-P', str(base_link_pose.get('qy', 0.0)),
            '-Y', str(base_link_pose.get('qz', 0.0)),
            '-W', str(base_link_pose.get('qw', 1.0)),
            '-static',
        ],
        output='screen',
    )

    # JOINT STATE BROADCASTER
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # MOVEIT MOVE_GROUP
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            planning_scene_monitor_parameters,
            planning_pipeline_config,
            {'use_sim_time': True},
        ],
    )

    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            ompl_planning_config,
            {'use_sim_time': True},
        ],
        condition=IfCondition(use_rviz),
    )

    # COLLISION OBJECTS
    add_collision_objects = Node(
        package='hold_and_weld_application',
        executable='add_collision_objects.py',
        name='add_collision_objects',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # MAGIC WAND (Interactive Markers)
    magic_wand = Node(
        package='hold_and_weld_application',
        executable='magic_wand.py',
        name='magic_wand',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # TIMING SEQUENCE
    delay_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster],
    )
    delay_move_group = TimerAction(period=8.0, actions=[move_group])
    delay_rviz = TimerAction(period=11.0, actions=[rviz])
    delay_add_collision_objects = TimerAction(period=13.0, actions=[add_collision_objects])
    delay_magic_wand = TimerAction(period=15.0, actions=[magic_wand])

    # ASSEMBLE LAUNCH DESCRIPTION
    nodes = [
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_child_link,
        spawn_base_link,
        delay_joint_state_broadcaster,
        delay_move_group,
        delay_rviz,
        delay_add_collision_objects,
        delay_magic_wand,
    ]

    return LaunchDescription(declared_arguments + nodes)
