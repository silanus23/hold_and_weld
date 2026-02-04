#!/usr/bin/env python3

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
"""Add collision objects to MoveIt planning scene from URDF files."""

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
import os
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import subprocess
import xml.etree.ElementTree as ET
import yaml


class AddCollisionObjects(Node):
    """Combined node to add both base_link and child_link from URDF files."""

    def __init__(self):
        """Initialize the collision objects node."""
        super().__init__('add_objects_to_scene')

        # Load configuration from YAML
        app_pkg = get_package_share_directory('hold_and_weld_application')
        desc_pkg = get_package_share_directory('hold_and_weld_description')
        objects_yaml_path = os.path.join(app_pkg, 'config', 'collision_objects', 'objects.yaml')
        
        with open(objects_yaml_path, 'r') as file:
            objects_yaml_dict = yaml.safe_load(file)
        objects_config = objects_yaml_dict.get('/**', {}).get('ros__parameters', {})

        # Get configuration
        frame_id = objects_config.get('frame_id', 'world')
        
        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )

        # Wait for subscribers
        self.get_logger().info('Waiting for collision_object subscribers')
        while self.collision_pub.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Add child_link from URDF
        child_link_config = objects_config.get('child_link', {})
        self.add_object_from_urdf(
            child_link_config.get('urdf_path', ''),
            child_link_config.get('id', 'child_link'),
            child_link_config.get('pose', {}),
            frame_id,
            desc_pkg
        )

        # Add base_link from URDF
        base_link_config = objects_config.get('base_link', {})
        self.add_object_from_urdf(
            base_link_config.get('urdf_path', ''),
            base_link_config.get('id', 'base_link'),
            base_link_config.get('pose', {}),
            frame_id,
            desc_pkg
        )

    def add_object_from_urdf(self, urdf_path, object_id, pose_config, frame_id, desc_pkg):
        """Add collision object by parsing URDF file."""
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

        # Create collision object
        collision_obj = CollisionObject()
        collision_obj.header = Header()
        collision_obj.header.frame_id = frame_id
        collision_obj.id = object_id

        # Extract collision geometry from first link
        link = root.find('.//link')
        if link is None:
            self.get_logger().error(f'No link found in URDF for {object_id}')
            return

        collision = link.find('collision')
        if collision is None:
            self.get_logger().error(f'No collision geometry found for {object_id}')
            return

        geometry = collision.find('geometry')
        if geometry is None:
            self.get_logger().error(f'No geometry found in collision for {object_id}')
            return

        # Parse geometry (box, sphere, cylinder)
        box = geometry.find('box')
        if box is not None:
            size_attr = box.get('size')
            if size_attr:
                sizes = [float(x) for x in size_attr.split()]
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = sizes
                collision_obj.primitives = [primitive]
        
        # Set pose
        pose = Pose()
        pose.position.x = pose_config.get('x', 0.0)
        pose.position.y = pose_config.get('y', 0.0)
        pose.position.z = pose_config.get('z', 0.0)
        pose.orientation.w = 1.0
        
        collision_obj.primitive_poses = [pose]
        collision_obj.operation = CollisionObject.ADD

        self.get_logger().info(
            f'Adding {object_id} to planning scene from URDF: {urdf_path}'
        )
        self.collision_pub.publish(collision_obj)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f'{object_id} added to planning scene successfully!')


def main(args=None):
    """Run the collision objects node."""
    rclpy.init(args=args)
    AddCollisionObjects()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
