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
"""
Finger visualizer — publishes gripper finger box markers in RViz.

Publishes markers for each sampled grasp, showing exactly where the fingers
will be placed on the part.

  - Spawns objects at their START pose (where the gripper picks them up).
  - Reads the latest grasp JSON from grasps/ by metadata.generated_at.
  - Publishes two CUBE markers per grasp (one per finger) on /grasp_markers.
"""

import colorsys
import glob
import json
import os
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
from moveit_msgs.msg import CollisionObject
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
    MarkerArray,
)
import yaml


# Grasps directories to search, in priority order:
#   1. Source tree — used when grasp_finder_node is run with --output pointing here
#   2. Install tree — default output location when --output is not passed
_WORKSPACE_ROOT = os.path.expanduser('~/ros2_yaskawa')
_GRASPS_DIRS = [
    os.path.join(
        _WORKSPACE_ROOT,
        'src/hold_and_weld/hold_and_weld_application/grasps',
    ),
    # Resolved at runtime in _visualize_latest_grasps() from ament_index
]


class FingerVisualizer(Node):
    """
    Publishes gripper finger box markers in RViz for each sampled grasp.

    Startup sequence
    ----------------
    1. Load objects.yaml to get start poses and URDF paths.
    2. Wait up to 5 s for a /collision_object subscriber (MoveIt).
    3. Spawn all objects from objects.yaml at their START poses in the planning
       scene (so the scene matches what the grasp sampler saw).
    4. Find the latest grasp JSON in grasps/ by metadata.generated_at.
    5. Publish two CUBE markers per grasp (finger_1, finger_2) on /grasp_markers.
    6. Spin forever (markers persist until the node dies).
    """

    def __init__(self):
        super().__init__('finger_visualizer')

        self.get_logger().info('=' * 50)
        self.get_logger().info('FINGER VISUALIZER STARTING')
        self.get_logger().info('=' * 50)

        app_pkg = get_package_share_directory('hold_and_weld_application')
        desc_pkg = get_package_share_directory('hold_and_weld_description')

        objects_yaml_path = os.path.join(
            app_pkg, 'config', 'collision_objects', 'objects.yaml'
        )
        with open(objects_yaml_path, 'r') as fh:
            objects_cfg = yaml.safe_load(fh).get('/**', {}).get(
                'ros__parameters', {}
            )

        self.frame_id = objects_cfg.get('frame_id', 'world')
        self._desc_pkg = desc_pkg

        self.collision_pub = self.create_publisher(
            CollisionObject, '/collision_object', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/grasp_markers', 10
        )
        self.marker_server = InteractiveMarkerServer(self, 'grasp_poses')

        self.get_logger().info(
            'Waiting for /collision_object subscriber (max 5 s)...'
        )
        for _ in range(50):
            if self.collision_pub.get_subscription_count() >= 1:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.collision_pub.get_subscription_count() < 1:
            self.get_logger().warn(
                'No /collision_object subscriber found — collision objects '
                'will not appear in MoveIt. Is move_group running?'
            )
        else:
            self._spawn_objects_at_start_pose(objects_cfg)

        self._visualize_latest_grasps()

    def _spawn_objects_at_start_pose(self, objects_cfg: dict):
        """Spawn all objects at their START poses."""
        for key, cfg in objects_cfg.items():
            if key == 'frame_id' or not isinstance(cfg, dict):
                continue

            urdf_rel = cfg.get('urdf_path', '')
            obj_id = cfg.get('id', key)

            # Start pose lives under cfg['pose'] (x/y/z flat dict) and
            # cfg['orientation'] (x/y/z/w flat dict) — matching objects.yaml.
            pose_cfg = cfg.get('pose', {})
            orient_cfg = cfg.get('orientation', {})

            if not pose_cfg:
                self.get_logger().warn(
                    f"No 'pose' defined for '{key}', skipping."
                )
                continue

            self.get_logger().info(
                f"Spawning '{obj_id}' at START pose "
                f"({pose_cfg.get('x', 0):.3f}, "
                f"{pose_cfg.get('y', 0):.3f}, "
                f"{pose_cfg.get('z', 0):.3f})"
            )
            self._spawn_collision_object(
                urdf_rel, obj_id, pose_cfg, orient_cfg
            )

    def _spawn_collision_object(
        self,
        urdf_rel: str,
        object_id: str,
        pose_cfg: dict,
        orient_cfg: dict,
    ):
        """Parse a xacro/URDF, extract collision geometry, publish to MoveIt."""
        full_urdf_path = os.path.join(self._desc_pkg, urdf_rel)

        # xacro → plain URDF string
        try:
            result = subprocess.run(
                ['xacro', full_urdf_path],
                capture_output=True, text=True, check=True,
            )
            urdf_content = result.stdout
        except subprocess.CalledProcessError as exc:
            self.get_logger().error(
                f"xacro failed for '{object_id}': {exc}"
            )
            return

        # Parse XML
        try:
            root = ET.fromstring(urdf_content)
        except ET.ParseError as exc:
            self.get_logger().error(
                f"URDF parse error for '{object_id}': {exc}"
            )
            return

        collision_obj = CollisionObject()
        collision_obj.header = Header()
        collision_obj.header.frame_id = self.frame_id
        collision_obj.id = object_id

        # Match the specific link by name; fall back to first link if not found
        link = root.find(f".//link[@name='{object_id}']")
        if link is None:
            link = root.find('.//link')
        if link is None:
            self.get_logger().error(f"No <link> in URDF for '{object_id}'")
            return

        # Iterate ALL <collision> tags so multi-primitive objects are fully represented
        collisions = link.findall('collision')
        if not collisions:
            self.get_logger().error(
                f"No collision geometry in URDF for '{object_id}'"
            )
            return

        for col in collisions:
            geometry = col.find('geometry')
            if geometry is None:
                continue

            primitive = self._geometry_to_primitive(geometry, object_id)
            if primitive is None:
                continue

            # Local collision origin offset (translation only; rpy ignored for primitives)
            local_x, local_y, local_z = 0.0, 0.0, 0.0
            origin = col.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0').split()
                if len(xyz) == 3:
                    local_x, local_y, local_z = float(xyz[0]), float(xyz[1]), float(xyz[2])

            pose = Pose()
            pose.position.x = float(pose_cfg.get('x', 0.0)) + local_x
            pose.position.y = float(pose_cfg.get('y', 0.0)) + local_y
            pose.position.z = float(pose_cfg.get('z', 0.0)) + local_z
            pose.orientation.x = float(orient_cfg.get('x', 0.0))
            pose.orientation.y = float(orient_cfg.get('y', 0.0))
            pose.orientation.z = float(orient_cfg.get('z', 0.0))
            pose.orientation.w = float(orient_cfg.get('w', 1.0))

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)

        if not collision_obj.primitives:
            self.get_logger().error(
                f"All collision geometries failed to parse for '{object_id}'"
            )
            return

        collision_obj.operation = CollisionObject.ADD

        self.collision_pub.publish(collision_obj)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f"'{object_id}' added to planning scene.")

    def _geometry_to_primitive(
        self, geometry: ET.Element, object_id: str
    ):
        """Convert a URDF <geometry> element to a MoveIt SolidPrimitive."""
        box = geometry.find('box')
        if box is not None:
            sizes = [float(v) for v in box.get('size', '0 0 0').split()]
            p = SolidPrimitive()
            p.type = SolidPrimitive.BOX
            p.dimensions = sizes
            return p

        cylinder = geometry.find('cylinder')
        if cylinder is not None:
            p = SolidPrimitive()
            p.type = SolidPrimitive.CYLINDER
            p.dimensions = [
                float(cylinder.get('length', 0.0)),
                float(cylinder.get('radius', 0.0)),
            ]
            return p

        sphere = geometry.find('sphere')
        if sphere is not None:
            p = SolidPrimitive()
            p.type = SolidPrimitive.SPHERE
            p.dimensions = [float(sphere.get('radius', 0.0))]
            return p

        self.get_logger().warn(
            f"Unknown geometry type for '{object_id}' — only box/cylinder/sphere supported."
        )
        return None

    def _visualize_latest_grasps(self):
        """Find the latest grasp JSON and publish all visualization markers."""
        # Build the list of directories to search, adding the install-tree
        # location resolved at runtime so we don't hardcode the install path.
        search_dirs = list(_GRASPS_DIRS)
        try:
            app_pkg = get_package_share_directory('hold_and_weld_application')
            install_grasps = os.path.join(app_pkg, 'grasps')
            if install_grasps not in search_dirs:
                search_dirs.append(install_grasps)
        except Exception:
            pass  # package not found — skip install-tree fallback

        json_files = []
        for d in search_dirs:
            if os.path.exists(d):
                found = glob.glob(os.path.join(d, '*.json'))
                if found:
                    self.get_logger().info(
                        f'Found {len(found)} JSON file(s) in {d}'
                    )
                json_files.extend(found)

        if not json_files:
            self.get_logger().error(
                f'No grasp JSON files found in any of: {search_dirs}'
            )
            return

        latest_path = None
        latest_data = None
        latest_time = None

        for path in json_files:
            try:
                with open(path, 'r') as fh:
                    data = json.load(fh)
                ts = data.get('metadata', {}).get('generated_at')
                if ts and (latest_time is None or ts > latest_time):
                    latest_time = ts
                    latest_path = path
                    latest_data = data
            except Exception as exc:
                self.get_logger().warn(f'Could not read {path}: {exc}')

        if latest_data is None:
            self.get_logger().error('No valid grasp JSON files found.')
            return

        self.get_logger().info(
            f'Loading: {os.path.basename(latest_path)} '
            f'(generated_at: {latest_time})'
        )

        # Sanity-check: must have a 'grasps' key, not 'seams' (trajectory JSON).
        if 'grasps' not in latest_data:
            self.get_logger().error(
                f'{os.path.basename(latest_path)} does not look like a grasp '
                'JSON (no "grasps" key). Did you accidentally point at a '
                'trajectory file?'
            )
            return

        self._publish_grasp_markers(latest_data)

    def _publish_grasp_markers(self, data: dict):
        """Build and publish MarkerArray + InteractiveMarkers for all grasps."""
        metadata = data.get('metadata', {})
        grasps = data.get('grasps', [])
        frame_id = metadata.get('coordinate_frame', self.frame_id)

        if not grasps:
            self.get_logger().warn('Grasp JSON contains no grasps.')
            return

        self.get_logger().info(
            f'{len(grasps)} grasp(s) in JSON — '
            f"source: {metadata.get('primary_source', 'unknown')}"
        )

        marker_array = MarkerArray()

        finger_length = metadata.get('finger_length', 0.05)

        for i, grasp in enumerate(grasps):
            markers = self._make_grasp_markers(grasp, i, frame_id, finger_length)
            marker_array.markers.extend(markers)

        self.marker_pub.publish(marker_array)

        self.get_logger().info(
            f'Published {len(marker_array.markers)} markers '
            f'(2 fingers + 2 tip spheres each) for {len(grasps)} grasp(s)'
        )

    @staticmethod
    def _quat_rotate(quat: list, v: list) -> list:
        """Rotate vector v by unit quaternion quat = [x, y, z, w]."""
        qx, qy, qz, qw = quat
        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (qy * v[2] - qz * v[1])
        ty = 2.0 * (qz * v[0] - qx * v[2])
        tz = 2.0 * (qx * v[1] - qy * v[0])
        # result = v + qw * t + cross(q.xyz, t)
        return [
            v[0] + qw * tx + qy * tz - qz * ty,
            v[1] + qw * ty + qz * tx - qx * tz,
            v[2] + qw * tz + qx * ty - qy * tx,
        ]

    def _make_grasp_markers(
        self, grasp: dict, index: int, frame_id: str, finger_length: float = 0.05
    ) -> list:
        """
        Return a list of Marker messages for one grasp.

        Finger geometry matches gripper_prefix.xacro exactly:
          - Box cross-section : 30 mm (X) × 40 mm (Y)  [local gripper frame]
          - Box length        : 200 mm (Z)  — finger_length from metadata
          - Finger origin offset in URDF: xyz="0 0 -0.1" meaning the box centre
            is 100 mm below the joint origin, so the tip is at z=-0.20 and the
            root is at z=0.00 relative to the joint.

        # In the gripper frame built by compute_gripper_transform:
          Y = grip axis (contact_1 → contact_2)
          Z = approach direction (outward from workpiece face, away from palm)
          X = Y × Z

        So -Z in gripper frame points inward (toward palm), and the finger tip
        sits at the contact point (z_local = 0), with the body extending
        finger_length along -Z.  Centre of the box is at finger_length/2
        along -Z from the contact point.

        Each finger also gets a SPHERE marker at its tip (the contact face) so
        the exact contact location is clearly visible.  Sphere radius equals
        half the smaller cross-section dimension (15 mm).
        """
        markers = []

        tcp = grasp.get('tcp_pose', {})
        quat = tcp.get('quaternion', [0.0, 0.0, 0.0, 1.0])

        # contact position arrays: [x, y, z]
        c1 = grasp.get('contact_1', {}).get('position', [0.0, 0.0, 0.0])
        c2 = grasp.get('contact_2', {}).get('position', [0.0, 0.0, 0.0])

        finger_x = 0.03
        finger_y = 0.04
        finger_l = float(finger_length)

        # Tip sphere: radius = half of the smaller cross-section side (15 mm)
        tip_radius = finger_x / 2.0   # 0.015 m

        # Direction fingers extend inward: local +Z of gripper frame in world.
        # compute_gripper_transform sets Z = Y × X (grip × approach), which
        # points toward the gripper body — i.e. away from the workpiece face.
        inward = self._quat_rotate(quat, [0.0, 0.0, -1.0])

        half = finger_l / 2.0

        f1 = Marker()
        f1.header = Header(frame_id=frame_id)
        f1.ns = 'grasp_finger_1'
        f1.id = index
        f1.type = Marker.CUBE
        f1.action = Marker.ADD
        f1.pose.position.x = c1[0] + half * inward[0]
        f1.pose.position.y = c1[1] + half * inward[1]
        f1.pose.position.z = c1[2] + half * inward[2]
        f1.pose.orientation.x = quat[0]
        f1.pose.orientation.y = quat[1]
        f1.pose.orientation.z = quat[2]
        f1.pose.orientation.w = quat[3]
        f1.scale.x = finger_x
        f1.scale.y = finger_y
        f1.scale.z = finger_l
        f1.color = ColorRGBA(r=1.0, g=0.55, b=0.1, a=0.75)
        f1.lifetime.sec = 0
        markers.append(f1)

        # Tip sphere: sits exactly at the contact point (no inward offset)
        s1 = Marker()
        s1.header = Header(frame_id=frame_id)
        s1.ns = 'grasp_tip_1'
        s1.id = index
        s1.type = Marker.SPHERE
        s1.action = Marker.ADD
        s1.pose.position.x = c1[0]
        s1.pose.position.y = c1[1]
        s1.pose.position.z = c1[2]
        s1.pose.orientation.x = quat[0]
        s1.pose.orientation.y = quat[1]
        s1.pose.orientation.z = quat[2]
        s1.pose.orientation.w = quat[3]
        s1.scale.x = tip_radius * 2.0
        s1.scale.y = tip_radius * 2.0
        s1.scale.z = tip_radius * 2.0
        s1.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.95)
        s1.lifetime.sec = 0
        markers.append(s1)

        # Box
        f2 = Marker()
        f2.header = Header(frame_id=frame_id)
        f2.ns = 'grasp_finger_2'
        f2.id = index
        f2.type = Marker.CUBE
        f2.action = Marker.ADD
        f2.pose.position.x = c2[0] + half * inward[0]
        f2.pose.position.y = c2[1] + half * inward[1]
        f2.pose.position.z = c2[2] + half * inward[2]
        f2.pose.orientation.x = quat[0]
        f2.pose.orientation.y = quat[1]
        f2.pose.orientation.z = quat[2]
        f2.pose.orientation.w = quat[3]
        f2.scale.x = finger_x
        f2.scale.y = finger_y
        f2.scale.z = finger_l
        f2.color = ColorRGBA(r=0.1, g=0.75, b=1.0, a=0.75)
        f2.lifetime.sec = 0
        markers.append(f2)

        # Tip sphere
        s2 = Marker()
        s2.header = Header(frame_id=frame_id)
        s2.ns = 'grasp_tip_2'
        s2.id = index
        s2.type = Marker.SPHERE
        s2.action = Marker.ADD
        s2.pose.position.x = c2[0]
        s2.pose.position.y = c2[1]
        s2.pose.position.z = c2[2]
        s2.pose.orientation.x = quat[0]
        s2.pose.orientation.y = quat[1]
        s2.pose.orientation.z = quat[2]
        s2.pose.orientation.w = quat[3]
        s2.scale.x = tip_radius * 2.0
        s2.scale.y = tip_radius * 2.0
        s2.scale.z = tip_radius * 2.0
        s2.color = ColorRGBA(r=0.0, g=0.95, b=1.0, a=0.95)
        s2.lifetime.sec = 0
        markers.append(s2)

        return markers

    def _register_interactive_marker(
        self, grasp: dict, index: int, frame_id: str
    ):
        """Register one InteractiveMarker so RViz shows hover descriptions."""
        tcp = grasp.get('tcp_pose', {})
        pos = tcp.get('position', [0.0, 0.0, 0.0])
        quat = tcp.get('quaternion', [0.0, 0.0, 0.0, 1.0])
        quality = grasp.get('quality_score', 0.0)
        opening_mm = grasp.get('gripper_opening', 0.0) * 1000.0

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame_id
        int_marker.name = f'grasp_{index}'
        int_marker.description = (
            f'Grasp #{index}\n'
            f'Quality : {quality:.3f}\n'
            f'Opening : {opening_mm:.1f} mm\n'
            f'Surf 1  : {grasp.get("contact_1", {}).get("surface_id", "?")}\n'
            f'Surf 2  : {grasp.get("contact_2", {}).get("surface_id", "?")}'
        )
        int_marker.pose.position.x = pos[0]
        int_marker.pose.position.y = pos[1]
        int_marker.pose.position.z = pos[2]
        int_marker.pose.orientation.x = quat[0]
        int_marker.pose.orientation.y = quat[1]
        int_marker.pose.orientation.z = quat[2]
        int_marker.pose.orientation.w = quat[3]

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.NONE

        # Y-axis finger-spread arrow (green, rotated 90° around Z in gripper frame)
        # Shows the grip axis direction for hover context in RViz.
        y_arrow = Marker()
        y_arrow.type = Marker.ARROW
        y_arrow.scale.x = 0.03
        y_arrow.scale.y = 0.005
        y_arrow.scale.z = 0.005
        y_arrow.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.9)
        y_arrow.pose.orientation.z = 0.7071068
        y_arrow.pose.orientation.w = 0.7071068
        control.markers.append(y_arrow)

        int_marker.controls.append(control)
        self.marker_server.insert(
            int_marker, feedback_callback=self._marker_feedback
        )

    def _quality_color(self, quality: float) -> ColorRGBA:
        """Map quality ∈ [0, 1] to a red→yellow→green color."""
        q = max(0.0, min(1.0, quality))
        r, g, b = colorsys.hsv_to_rgb(q * 0.33, 1.0, 1.0)
        return ColorRGBA(r=r, g=g, b=b, a=0.9)

    def _marker_feedback(self, feedback):
        self.get_logger().info(
            f'Selected: {feedback.marker_name} at '
            f'({feedback.pose.position.x:.3f}, '
            f'{feedback.pose.position.y:.3f}, '
            f'{feedback.pose.position.z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FingerVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
