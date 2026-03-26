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
"""Visualize grasp results from JSON in RViz."""

import argparse
import colorsys
import glob
import json
import os

from geometry_msgs.msg import Point, Vector3
from interactive_markers import InteractiveMarkerServer
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
    MarkerArray,
)


class GraspVisualizer(Node):
    """Node to visualize grasp results from JSON files."""

    def __init__(self, json_path: str = None):
        """Initialize the grasp visualizer node."""
        super().__init__('grasp_visualizer')

        self.get_logger().info('=' * 50)
        self.get_logger().info('GRASP VISUALIZER STARTING')
        self.get_logger().info('=' * 50)

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, '/grasp_markers', 10
        )

        # Interactive marker server for detailed inspection
        self.marker_server = InteractiveMarkerServer(self, 'grasp_poses')

        # Load and visualize grasps
        if json_path:
            self.load_and_visualize(json_path)
        else:
            self.load_and_visualize_latest()

    def load_and_visualize_latest(self):
        """Load the latest grasp JSON and visualize."""
        # Look in application package grasps folder (source directory)
        workspace_root = os.path.expanduser('~/ros2_yaskawa')
        grasps_dir = os.path.join(
            workspace_root,
            'src/hold_and_weld/hold_and_weld_application/grasps'
        )

        if not os.path.exists(grasps_dir):
            self.get_logger().error(f'Grasps directory not found: {grasps_dir}')
            return

        self.get_logger().info(f'Searching for JSON files in: {grasps_dir}')

        json_files = glob.glob(os.path.join(grasps_dir, '*.json'))
        if not json_files:
            self.get_logger().error('No JSON files found in grasps directory!')
            return

        # Find latest by metadata generated_at
        latest_json = None
        latest_time = None

        for json_file in json_files:
            try:
                with open(json_file, 'r') as f:
                    data = json.load(f)
                generated_at = data.get('metadata', {}).get('generated_at')
                if generated_at and (
                    latest_time is None or generated_at > latest_time
                ):
                    latest_time = generated_at
                    latest_json = (json_file, data)
            except Exception as e:
                self.get_logger().warn(f'Could not read {json_file}: {e}')
                continue

        if latest_json is None:
            self.get_logger().error('No valid grasp JSON files found!')
            return

        json_path, data = latest_json
        self.get_logger().info(
            f'Loading: {os.path.basename(json_path)} '
            f'(generated_at: {latest_time})'
        )

        self.visualize_grasps(data)

    def load_and_visualize(self, json_path: str):
        """Load specific JSON file and visualize."""
        if not os.path.exists(json_path):
            self.get_logger().error(f'JSON file not found: {json_path}')
            return

        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Failed to load JSON: {e}')
            return

        self.get_logger().info(f'Loaded: {json_path}')
        self.visualize_grasps(data)

    def visualize_grasps(self, data: dict):
        """Create visualization markers for grasps."""
        metadata = data.get('metadata', {})
        grasps = data.get('grasps', [])

        if not grasps:
            self.get_logger().warn('No grasps found in JSON!')
            return

        frame_id = metadata.get('coordinate_frame', 'world')
        self.get_logger().info(f'Coordinate frame: {frame_id}')
        self.get_logger().info(f'Number of grasps: {len(grasps)}')

        # Print statistics if available
        if metadata.get('num_contact_pairs'):
            self.get_logger().info(
                f"Statistics: {metadata.get('num_surfaces_valid', '?')} valid surfaces, "
                f"{metadata.get('num_contact_pairs', '?')} contact pairs, "
                f"{metadata.get('num_candidates', '?')} candidates"
            )

        # Create marker array for all grasps
        marker_array = MarkerArray()

        for i, grasp in enumerate(grasps):
            # Create markers for this grasp
            grasp_markers = self.create_grasp_markers(grasp, i, frame_id)
            marker_array.markers.extend(grasp_markers)

            # Create interactive marker for detailed inspection
            self.create_interactive_marker(grasp, i, frame_id)

        # Publish marker array
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} markers')

        # Apply interactive markers
        self.marker_server.applyChanges()
        self.get_logger().info(
            f'Created {len(grasps)} interactive markers '
            '(topic: /grasp_poses/update)'
        )

    def create_grasp_markers(
        self, grasp: dict, index: int, frame_id: str
    ) -> list:
        """Create visualization markers for a single grasp."""
        markers = []
        quality = grasp.get('quality_score', 0.0)

        # Color based on quality (green = high, red = low)
        color = self.quality_to_color(quality)

        tcp_pose = grasp.get('tcp_pose', {})
        position = tcp_pose.get('position', [0, 0, 0])
        quaternion = tcp_pose.get('quaternion', [0, 0, 0, 1])

        contact_1 = grasp.get('contact_1', {}).get('position', [0, 0, 0])
        contact_2 = grasp.get('contact_2', {}).get('position', [0, 0, 0])

        # TCP frame marker (coordinate axes)
        tcp_marker = Marker()
        tcp_marker.header = Header(frame_id=frame_id)
        tcp_marker.ns = 'grasp_tcp'
        tcp_marker.id = index
        tcp_marker.type = Marker.ARROW
        tcp_marker.action = Marker.ADD
        tcp_marker.pose.position.x = position[0]
        tcp_marker.pose.position.y = position[1]
        tcp_marker.pose.position.z = position[2]
        tcp_marker.pose.orientation.x = quaternion[0]
        tcp_marker.pose.orientation.y = quaternion[1]
        tcp_marker.pose.orientation.z = quaternion[2]
        tcp_marker.pose.orientation.w = quaternion[3]
        tcp_marker.scale = Vector3(x=0.05, y=0.008, z=0.008)
        tcp_marker.color = color
        tcp_marker.lifetime.sec = 0  # Persistent
        markers.append(tcp_marker)

        # Contact point 1 sphere
        c1_marker = Marker()
        c1_marker.header = Header(frame_id=frame_id)
        c1_marker.ns = 'grasp_contact_1'
        c1_marker.id = index
        c1_marker.type = Marker.SPHERE
        c1_marker.action = Marker.ADD
        c1_marker.pose.position.x = contact_1[0]
        c1_marker.pose.position.y = contact_1[1]
        c1_marker.pose.position.z = contact_1[2]
        c1_marker.pose.orientation.w = 1.0
        c1_marker.scale = Vector3(x=0.01, y=0.01, z=0.01)
        c1_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # Orange
        markers.append(c1_marker)

        # Contact point 2 sphere
        c2_marker = Marker()
        c2_marker.header = Header(frame_id=frame_id)
        c2_marker.ns = 'grasp_contact_2'
        c2_marker.id = index
        c2_marker.type = Marker.SPHERE
        c2_marker.action = Marker.ADD
        c2_marker.pose.position.x = contact_2[0]
        c2_marker.pose.position.y = contact_2[1]
        c2_marker.pose.position.z = contact_2[2]
        c2_marker.pose.orientation.w = 1.0
        c2_marker.scale = Vector3(x=0.01, y=0.01, z=0.01)
        c2_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.9)  # Blue
        markers.append(c2_marker)

        # Line between contacts (gripper opening)
        line_marker = Marker()
        line_marker.header = Header(frame_id=frame_id)
        line_marker.ns = 'grasp_opening'
        line_marker.id = index
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.003  # Line width
        line_marker.color = color
        line_marker.color.a = 0.6
        line_marker.points = [
            Point(x=contact_1[0], y=contact_1[1], z=contact_1[2]),
            Point(x=contact_2[0], y=contact_2[1], z=contact_2[2]),
        ]
        markers.append(line_marker)

        # Text label with quality score (only for top grasps)
        if index < 10:
            text_marker = Marker()
            text_marker.header = Header(frame_id=frame_id)
            text_marker.ns = 'grasp_label'
            text_marker.id = index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = position[0]
            text_marker.pose.position.y = position[1]
            text_marker.pose.position.z = position[2] + 0.03
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.015  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f'#{index} q={quality:.2f}'
            markers.append(text_marker)

        return markers

    def create_interactive_marker(
        self, grasp: dict, index: int, frame_id: str
    ):
        """Create interactive marker for detailed grasp inspection."""
        tcp_pose = grasp.get('tcp_pose', {})
        position = tcp_pose.get('position', [0, 0, 0])
        quaternion = tcp_pose.get('quaternion', [0, 0, 0, 1])
        quality = grasp.get('quality_score', 0.0)
        opening = grasp.get('gripper_opening', 0.0)

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = f'grasp_{index}'
        int_marker.description = (
            f'Grasp #{index}\n'
            f'Quality: {quality:.3f}\n'
            f'Opening: {opening*1000:.1f}mm'
        )
        int_marker.pose.position.x = position[0]
        int_marker.pose.position.y = position[1]
        int_marker.pose.position.z = position[2]
        int_marker.pose.orientation.x = quaternion[0]
        int_marker.pose.orientation.y = quaternion[1]
        int_marker.pose.orientation.z = quaternion[2]
        int_marker.pose.orientation.w = quaternion[3]

        # Visual control (gripper frame visualization)
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.NONE

        # Z-axis (approach direction)
        z_arrow = Marker()
        z_arrow.type = Marker.ARROW
        z_arrow.scale = Vector3(x=0.04, y=0.006, z=0.006)
        z_arrow.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)
        z_arrow.pose.orientation.w = 1.0
        control.markers.append(z_arrow)

        # Y-axis (finger direction)
        y_arrow = Marker()
        y_arrow.type = Marker.ARROW
        y_arrow.scale = Vector3(x=0.03, y=0.005, z=0.005)
        y_arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        # Rotate to point along Y
        y_arrow.pose.orientation.x = 0.0
        y_arrow.pose.orientation.y = 0.0
        y_arrow.pose.orientation.z = 0.7071
        y_arrow.pose.orientation.w = 0.7071
        control.markers.append(y_arrow)

        int_marker.controls.append(control)

        self.marker_server.insert(int_marker, feedback_callback=self.marker_feedback)

    def marker_feedback(self, feedback):
        """Handle interactive marker feedback."""
        self.get_logger().info(
            f'Selected: {feedback.marker_name} at '
            f'({feedback.pose.position.x:.3f}, '
            f'{feedback.pose.position.y:.3f}, '
            f'{feedback.pose.position.z:.3f})'
        )

    def quality_to_color(self, quality: float) -> ColorRGBA:
        """Convert quality score (0-1) to color (red-yellow-green)."""
        # Clamp quality to [0, 1]
        q = max(0.0, min(1.0, quality))

        # HSV: 0 = red, 0.33 = green
        # Map quality 0->1 to hue 0->0.33
        hue = q * 0.33
        r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)

        return ColorRGBA(r=r, g=g, b=b, a=0.9)


def main(args=None):
    """Run the grasp visualizer node."""
    parser = argparse.ArgumentParser(description='Visualize grasp results in RViz')
    parser.add_argument(
        '--json', '-j',
        type=str,
        default=None,
        help='Path to grasp JSON file (default: latest in grasps folder)'
    )

    # Parse known args to handle ROS2 args
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    visualizer = GraspVisualizer(json_path=parsed_args.json)

    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()