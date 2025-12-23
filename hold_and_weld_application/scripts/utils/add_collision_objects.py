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

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

# Combined node to add both cube and workpiece
class AddCollisionObjects(Node):
    def __init__(self):
        super().__init__("add_objects_to_scene")

        # Cube parameters
        self.declare_parameter("cube_size", 0.25)
        self.declare_parameter("cube_x", 1.2)
        self.declare_parameter("cube_y", 0.3)
        self.declare_parameter("cube_z", 0.125)

        # Workpiece parameters
        self.declare_parameter("length", 0.80)
        self.declare_parameter("width", 0.40)
        self.declare_parameter("height", 0.30)
        self.declare_parameter("workpiece_x", 1.2)
        self.declare_parameter("workpiece_y", -0.5)
        self.declare_parameter("workpiece_z", 0.65)

        self.declare_parameter("frame_id", "world")

        # Get parameters
        cube_size = self.get_parameter("cube_size").value
        cube_x = self.get_parameter("cube_x").value
        cube_y = self.get_parameter("cube_y").value
        cube_z = self.get_parameter("cube_z").value

        length = self.get_parameter("length").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        workpiece_x = self.get_parameter("workpiece_x").value
        workpiece_y = self.get_parameter("workpiece_y").value
        workpiece_z = self.get_parameter("workpiece_z").value

        frame_id = self.get_parameter("frame_id").value

        # Create publisher
        self.collision_pub = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        # Wait for subscribers
        self.get_logger().info("Waiting for collision_object subscribers")
        while self.collision_pub.get_subscription_count() < 1:
            rclpy.spin_once(self, timeout_sec=0.1)

        # --- Add Cube ---
        cube_obj = CollisionObject()
        cube_obj.header = Header()
        cube_obj.header.frame_id = frame_id
        cube_obj.id = "target_cube"

        cube_primitive = SolidPrimitive()
        cube_primitive.type = SolidPrimitive.BOX
        cube_primitive.dimensions = [cube_size, cube_size, cube_size]

        cube_pose = Pose()
        cube_pose.position.x = cube_x
        cube_pose.position.y = cube_y
        cube_pose.position.z = cube_z
        cube_pose.orientation.w = 1.0

        cube_obj.primitives = [cube_primitive]
        cube_obj.primitive_poses = [cube_pose]
        cube_obj.operation = CollisionObject.ADD

        self.get_logger().info(
            f"Adding cube to planning scene: size={cube_size}, position=({cube_x}, {cube_y}, {cube_z})"
        )
        self.collision_pub.publish(cube_obj)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("Cube added to planning scene successfully!")

        # --- Add Workpiece ---
        workpiece_obj = CollisionObject()
        workpiece_obj.header = Header()
        workpiece_obj.header.frame_id = frame_id
        workpiece_obj.id = "workpiece"

        workpiece_primitive = SolidPrimitive()
        workpiece_primitive.type = SolidPrimitive.BOX
        workpiece_primitive.dimensions = [length, width, height]

        workpiece_pose = Pose()
        workpiece_pose.position.x = workpiece_x
        workpiece_pose.position.y = workpiece_y
        workpiece_pose.position.z = workpiece_z
        workpiece_pose.orientation.w = 1.0

        workpiece_obj.primitives = [workpiece_primitive]
        workpiece_obj.primitive_poses = [workpiece_pose]
        workpiece_obj.operation = CollisionObject.ADD

        self.get_logger().info(
            f"Adding workpiece to planning scene: size=({length}, {width}, {height}), position=({workpiece_x}, {workpiece_y}, {workpiece_z})"
        )
        self.collision_pub.publish(workpiece_obj)
        rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info("Workpiece added to planning scene successfully!")


def main(args=None):
    rclpy.init(args=args)
    node = AddCollisionObjects()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
