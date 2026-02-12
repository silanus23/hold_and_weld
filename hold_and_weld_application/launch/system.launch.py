# Copyright 2025 Berkan Tali
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
Complete hold_and_weld system launch file.

Launches: Gazebo, MoveIt, RViz, and application nodes.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def quaternion_to_euler(qx, qy, qz, qw):
    """Convert quaternion to euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def generate_launch_description():
    """Launch main system with full stack."""
    # PACKAGE DIRECTORIES
    app_pkg = get_package_share_directory('hold_and_weld_application')
    desc_pkg = get_package_share_directory('hold_and_weld_description')

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
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Launch Gazebo with GUI (set to false for headless mode)',
        ),
    ]

    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')

    # FILE PATHS
    controller_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'config', 'controllers.yaml']
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'worlds', 'bringup_world.world']
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('hold_and_weld_description'), 'rviz', 'moveit.rviz']
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
            ' ',
            'controller_config_file:=',
            controller_config,
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

    # MOVEIT CONFIGURATION (Load separate YAML files)
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

    moveit_controllers_yaml_path = os.path.join(
        app_pkg, 'config', 'system', 'moveit_controllers.yaml'
    )
    with open(moveit_controllers_yaml_path, 'r') as file:
        moveit_controllers_yaml_dict = yaml.safe_load(file)
        moveit_controllers_config = moveit_controllers_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

    # MoveIt configuration parameters
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

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

    # OBJECT DESCRIPTIONS (CHILD_LINK & BASE_LINK)
    # Extract all object parameters first
    child_link_urdf_path = objects_config.get('child_link', {}).get('urdf_path', '')
    child_link_spawn_name = objects_config.get('child_link', {}).get('spawn_name', 'child_link')
    child_link_pose = objects_config.get('child_link', {}).get('pose', {})

    base_link_urdf_path = objects_config.get('base_link', {}).get('urdf_path', '')
    base_link_spawn_name = objects_config.get('base_link', {}).get('spawn_name', 'base_link')
    base_link_pose = objects_config.get('base_link', {}).get('pose', {})

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

    # GAZEBO SIMULATION

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

    # ROS-GAZEBO BRIDGE
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ('/world/default/model/dual_arm_system/joint_state'
             '@sensor_msgs/msg/JointState[gz.msgs.Model'),
            f'/model/{child_link_spawn_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            f'/model/{base_link_spawn_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/world/default/clock', '/clock'),
            ('/world/default/model/dual_arm_system/joint_state', '/joint_states'),
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
        name='spawn_dual_gp25_system',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'dual_gp25_system',
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # Convert child_link quaternion to euler
    child_roll, child_pitch, child_yaw = quaternion_to_euler(
        child_link_pose.get('qx', 0.0),
        child_link_pose.get('qy', 0.0),
        child_link_pose.get('qz', 0.0),
        child_link_pose.get('qw', 1.0)
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
    )

    # Convert base_link quaternion to euler
    base_roll, base_pitch, base_yaw = quaternion_to_euler(
        base_link_pose.get('qx', 0.0),
        base_link_pose.get('qy', 0.0),
        base_link_pose.get('qz', 0.0),
        base_link_pose.get('qw', 1.0)
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
    )

    # CONTROLLER SPAWNERS
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

    robot1_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robot1_arm_controller',
        arguments=[
            'robot1_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    robot2_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_robot2_arm_controller',
        arguments=[
            'robot2_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_gripper_controller',
        arguments=[
            'gripper_controller',
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
            trajectory_execution,
            {
                'moveit_simple_controller_manager': moveit_controllers_config.get(
                    'moveit_simple_controller_manager', {}
                )
            },
            {
                'moveit_controller_manager': moveit_controllers_config.get(
                    'moveit_controller_manager',
                    'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                )
            },
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

    add_collision_objects = Node(
        package='hold_and_weld_application',
        executable='add_collision_objects.py',
        name='add_collision_objects',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN']
    )

    # APPLICATION NODES
    welder_action_server = Node(
        package='hold_and_weld_application',
        executable='welder_server',
        name='welder_action_server',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {
                'welder_group_name': 'robot2_gp25_welder_arm',
                'auto_load_latest': True,
                'use_sim_time': True,
            }
        ],
    )

    gripper_action_server = Node(
        package='hold_and_weld_application',
        executable='gripper_server',
        name='gripper_action_server',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {
                'arm_group_name': 'robot1_gp25_arm',
                'gripper_controller_topic': '/gripper_controller/joint_trajectory',
                'use_sim_time': True,
                'auto_trigger': False,
            }
        ],
    )

    coordinator_node = Node(
        package='hold_and_weld_application',
        executable='dual_robot_coordinator',
        name='dual_robot_coordinator',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            {'use_sim_time': True},
        ],
    )

    delay_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster],
    )

    delay_robot1_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot1_arm_controller],
        )
    )

    delay_robot2_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot1_arm_controller,
            on_exit=[robot2_arm_controller],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot2_arm_controller,
            on_exit=[gripper_controller],
        )
    )

    # TIMING SEQUENCE (Timer-Based Application Nodes)
    delay_move_group = TimerAction(period=10.0, actions=[move_group])
    delay_rviz = TimerAction(period=13.0, actions=[rviz])
    delay_add_collision_objects = TimerAction(period=15.0, actions=[add_collision_objects])
    delay_welder_server = TimerAction(period=18.0, actions=[welder_action_server])
    delay_gripper_server = TimerAction(period=20.0, actions=[gripper_action_server])
    delay_coordinator = TimerAction(period=30.0, actions=[coordinator_node])

    # ASSEMBLE LAUNCH DESCRIPTION
    nodes = [
        gz_sim,
        ros_gz_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_child_link,
        spawn_base_link,
        delay_joint_state_broadcaster,
        delay_robot1_arm_controller,
        delay_robot2_arm_controller,
        delay_gripper_controller,
        delay_move_group,
        delay_rviz,
        delay_add_collision_objects,
        delay_welder_server,
        delay_gripper_server,
        delay_coordinator,
    ]

    return LaunchDescription(declared_arguments + nodes)
