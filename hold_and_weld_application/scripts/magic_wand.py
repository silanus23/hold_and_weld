#!/usr/bin/env python3

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
"""Magic wand to spawn child_link at end_pose and visualize JSON surfaces."""

import glob
import json
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from interactive_markers import InteractiveMarkerServer
from moveit_msgs.msg import CollisionObject
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
import yaml


class MagicWand(Node):
    """Magic wand node to manage collision objects and interactive markers."""

    def __init__(self):
        """Initialize the magic wand node."""
        super().__init__('magic_wand')

        self.get_logger().info('='*50)
        self.get_logger().info('MAGIC WAND STARTING')
        self.get_logger().info('='*50)

        # Load configuration from YAML
        app_pkg = get_package_share_directory('hold_and_weld_application')
        desc_pkg = get_package_share_directory('hold_and_weld_description')
        objects_yaml_path = os.path.join(
            app_pkg, 'config', 'collision_objects', 'objects.yaml'
        )

        with open(objects_yaml_path, 'r') as file:
            objects_yaml_dict = yaml.safe_load(file)
        objects_config = objects_yaml_dict.get('/**', {}).get(
            'ros__parameters', {}
        )

        self.frame_id = objects_config.get('frame_id', 'world')
        child_link_config = objects_config.get('child_link', {})
        base_link_config = objects_config.get('base_link', {})

        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )

        self.get_logger().info(
            'Creating interactive marker server for torch tips...'
        )
        self.marker_server = InteractiveMarkerServer(self, 'torch_tips')

        self.get_logger().info(
            'Waiting for collision_object subscribers (max 5 seconds)...'
        )
        wait_count = 0
        max_wait = 50
        while (
            self.collision_pub.get_subscription_count() < 1
            and wait_count < max_wait
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_count += 1

        if self.collision_pub.get_subscription_count() < 1:
            self.get_logger().warn(
                'No collision_object subscribers found. '
                'Skipping child_link spawn. Launch MoveIt to enable '
                'collision objects.'
            )
        else:
            base_pose = base_link_config.get('pose', {})
            if base_pose:
                self.get_logger().info('Spawning base_link')
                self.spawn_collision_object(
                    base_link_config.get('urdf_path', ''),
                    base_link_config.get('id', 'base_link'),
                    base_link_config,
                    desc_pkg
                )

            child_pose = child_link_config.get('pose', {})
            end_pose = child_link_config.get('end_pose', {})
            if child_pose:
                self.get_logger().info('Spawning child_link at initial pose')
                self.spawn_collision_object(
                    child_link_config.get('urdf_path', ''),
                    child_link_config.get('id', 'child_link'),
                    child_link_config,
                    desc_pkg
                )
            else:
                self.get_logger().warn('No initial pose defined for child_link')

            if end_pose:
                self.get_logger().info('Moving child_link to end_pose')
                self.move_collision_object(
                    child_link_config.get('id', 'child_link'),
                    end_pose
                )
            else:
                self.get_logger().warn('No end_pose defined for child_link')

        # Load and visualize torch tips from latest trajectory JSON
        self.load_and_visualize_latest_json()

    def spawn_collision_object(
        self, urdf_path, object_id, object_config, desc_pkg
    ):
        """Spawn collision object at its initial pose using ADD operation."""
        pose_config = object_config.get('pose', {})
        orientation_config = object_config.get('orientation', {})
        full_urdf_path = os.path.join(desc_pkg, urdf_path)

        # Process xacro to get URDF
        try:
            result = subprocess.run(
                ['xacro', full_urdf_path],
                capture_output=True,
                text=True,
                check=True
            )
            urdf_content = result.stdout
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to process xacro: {e}')
            return

        # Parse URDF XML
        try:
            root = ET.fromstring(urdf_content)
        except ET.ParseError as e:
            self.get_logger().error(f'Failed to parse URDF: {e}')
            return

        collision_obj = CollisionObject()
        collision_obj.header = Header()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = object_id

        # Extract collision geometry from first link
        link = root.find('.//link')
        if link is None:
            self.get_logger().error(
                f'No link found in URDF for {object_id}'
            )
            return

        collision = link.find('collision')
        if collision is None:
            self.get_logger().error(
                f'No collision geometry found for {object_id}'
            )
            return

        geometry = collision.find('geometry')
        if geometry is None:
            self.get_logger().error(
                f'No geometry found in collision for {object_id}'
            )
            return

        box = geometry.find('box')
        if box is not None:
            size_attr = box.get('size')
            if size_attr:
                sizes = [float(x) for x in size_attr.split()]
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = sizes
                collision_obj.primitives = [primitive]

        pose = Pose()
        pose.position.x = pose_config.get('x', 0.0)
        pose.position.y = pose_config.get('y', 0.0)
        pose.position.z = pose_config.get('z', 0.0)
        pose.orientation.x = orientation_config.get('x', 0.0)
        pose.orientation.y = orientation_config.get('y', 0.0)
        pose.orientation.z = orientation_config.get('z', 0.0)
        pose.orientation.w = orientation_config.get('w', 1.0)

        collision_obj.primitive_poses = [pose]
        collision_obj.operation = CollisionObject.ADD

        self.get_logger().info(
            f'Spawning {object_id} at initial pose: '
            f'position({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}), '
            f'orientation({pose.orientation.x:.3f}, '
            f'{pose.orientation.y:.3f}, {pose.orientation.z:.3f}, '
            f'{pose.orientation.w:.3f})'
        )

        if self.collision_pub.get_subscription_count() > 0:
            self.collision_pub.publish(collision_obj)
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info(f'{object_id} spawned successfully!')
        else:
            self.get_logger().warn(
                f'{object_id} not spawned - no MoveIt subscribers available'
            )

    def move_collision_object(self, object_id, end_pose_config):
        """Move an existing collision object to end_pose using MOVE operation."""
        position = end_pose_config.get('position', {})
        orientation = end_pose_config.get('orientation', {})

        pose = Pose()
        pose.position.x = position.get('x', 0.0)
        pose.position.y = position.get('y', 0.0)
        pose.position.z = position.get('z', 0.0)
        pose.orientation.x = orientation.get('x', 0.0)
        pose.orientation.y = orientation.get('y', 0.0)
        pose.orientation.z = orientation.get('z', 0.0)
        pose.orientation.w = orientation.get('w', 1.0)

        collision_obj = CollisionObject()
        collision_obj.header = Header()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = object_id
        collision_obj.primitive_poses = [pose]
        collision_obj.operation = CollisionObject.MOVE

        self.get_logger().info(
            f'Moving {object_id} to end_pose: '
            f'position({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}), '
            f'orientation({pose.orientation.x:.3f}, '
            f'{pose.orientation.y:.3f}, {pose.orientation.z:.3f}, '
            f'{pose.orientation.w:.3f})'
        )

        if self.collision_pub.get_subscription_count() > 0:
            self.collision_pub.publish(collision_obj)
            rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info(f'{object_id} moved to end_pose successfully!')
        else:
            self.get_logger().warn(
                f'{object_id} not moved - no MoveIt subscribers available'
            )

    def load_and_visualize_latest_json(self):
        """Load the latest trajectory JSON and create torch tip markers."""
        # Point directly to source trajectories directory
        # This works whether running from source or install
        workspace_root = os.path.expanduser('~/ros2_yaskawa')
        trajectories_dir = os.path.join(
            workspace_root,
            'src/hold_and_weld/hold_and_weld_application/trajectories'
        )

        if not os.path.exists(trajectories_dir):
            self.get_logger().error(
                f'Trajectories directory not found: {trajectories_dir}'
            )
            return

        self.get_logger().info(
            f'Searching for JSON files in: {trajectories_dir}'
        )

        json_files = glob.glob(os.path.join(trajectories_dir, '*.json'))
        if not json_files:
            self.get_logger().error('No JSON files found in trajectories directory!')
            return

        # Load all files and pick latest by metadata generated_at
        latest_json = None
        latest_time = None

        for json_file in json_files:
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                generated_at = data.get('metadata', {}).get('generated_at')
                if generated_at and (latest_time is None or generated_at > latest_time):
                    latest_time = generated_at
                    latest_json = (json_file, data)
            except Exception as e:
                self.get_logger().warn(f'Could not read {json_file}: {e}')
                continue

        if latest_json is None:
            self.get_logger().error('No valid trajectory JSON files found!')
            return

        json_path, data = latest_json
        self.get_logger().info(
            f'Loading latest JSON: {os.path.basename(json_path)} (generated_at: {latest_time})'
        )

        if 'seams' in data:
            self.get_logger().info(
                'Detected trajectory JSON - creating torch tip markers'
            )
            self.create_torch_tip_markers(data)
        else:
            self.get_logger().error(
                'Not a trajectory JSON! '
                'Use add_collision_objects.py for surfaces.'
            )

    def create_torch_tip_markers(self, data):
        """Create interactive markers for torch tips from trajectory JSON."""
        seams = data.get('seams', {})
        metadata = data.get('metadata', {})

        if not seams:
            self.get_logger().error('No seams found in JSON!')
            return

        coordinate_frame = metadata.get('coordinate_system', 'world')
        self.get_logger().info(f'Using coordinate frame: {coordinate_frame}')
        self.get_logger().info(f'Found {len(seams)} seams in JSON')

        marker_count = 0
        for seam_name, seam_data in seams.items():
            poses = seam_data.get('poses', [])
            self.get_logger().info(
                f'Processing {seam_name}: {len(poses)} poses'
            )

            sample_step = max(1, len(poses) // 20)
            self.get_logger().info(f'Sampling every {sample_step} poses')

            for i, pose_data in enumerate(poses[::sample_step]):
                position = pose_data.get('position', [0, 0, 0])
                quaternion = pose_data.get('quaternion', [0, 0, 0, 1])

                int_marker = InteractiveMarker()
                int_marker.header.frame_id = coordinate_frame
                int_marker.name = f'{seam_name}_pose_{i*sample_step}'
                int_marker.description = (
                    f'{seam_name} Torch Tip {i*sample_step}'
                )
                int_marker.pose.position.x = float(position[0])
                int_marker.pose.position.y = float(position[1])
                int_marker.pose.position.z = float(position[2])
                int_marker.pose.orientation.x = float(quaternion[0])
                int_marker.pose.orientation.y = float(quaternion[1])
                int_marker.pose.orientation.z = float(quaternion[2])
                int_marker.pose.orientation.w = float(quaternion[3])

                marker_control = InteractiveMarkerControl()
                marker_control.always_visible = True
                marker_control.interaction_mode = InteractiveMarkerControl.NONE

                axis_marker = Marker()
                axis_marker.type = Marker.ARROW
                axis_marker.scale.x = 0.01
                axis_marker.scale.y = 0.015
                axis_marker.scale.z = 0.015

                # Color based on seam (rainbow effect)
                color = ColorRGBA()
                hue = (hash(seam_name) % 360) / 360.0
                color.r, color.g, color.b = self.hsv_to_rgb(hue, 1.0, 1.0)
                color.a = 0.8
                axis_marker.color = color

                start_point = Point()
                start_point.x, start_point.y, start_point.z = 0.0, 0.0, -0.1
                end_point = Point()
                end_point.x, end_point.y, end_point.z = 0.0, 0.0, 0.0
                axis_marker.points = [start_point, end_point]

                marker_control.markers.append(axis_marker)
                int_marker.controls.append(marker_control)

                self.marker_server.insert(
                    int_marker, feedback_callback=self.marker_feedback
                )
                marker_count += 1

        self.get_logger().info(
            f'Inserted {marker_count} markers, calling applyChanges()...'
        )
        self.marker_server.applyChanges()
        self.get_logger().info(
            f'Created {marker_count} torch tip markers and applied changes!'
        )
        self.get_logger().info(
            'Check RViz InteractiveMarkers display with topic: '
            '/torch_tips/update'
        )

    def hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB."""
        import colorsys
        return colorsys.hsv_to_rgb(h, s, v)

    def marker_feedback(self, feedback):
        """Handle interactive marker feedback."""
        self.get_logger().info(
            f'Feedback from marker {feedback.marker_name}: '
            f'position({feedback.pose.position.x:.3f}, '
            f'{feedback.pose.position.y:.3f}, {feedback.pose.position.z:.3f})'
        )


def main(args=None):
    """Run the magic wand node."""
    rclpy.init(args=args)
    magic_wand = MagicWand()

    try:
        rclpy.spin(magic_wand)
    except KeyboardInterrupt:
        pass
    finally:
        magic_wand.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
