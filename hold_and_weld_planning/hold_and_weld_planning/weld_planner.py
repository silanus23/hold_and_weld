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


"""Weld path planner - handles all geometric calculations for weld trajectory generation."""

import numpy as np
from scipy.spatial.transform import Rotation


class WeldPlanner:
    """
    Generate weld poses for seams based on joint type and weld parameters.

    Modifies seam objects in-place following the Mutable State pattern.
    """

    SUPPORTED_JOINT_TYPES = ['t_joint', 'butt_joint', 'corner_joint', 'lap_joint', 'edge_joint']

    def __init__(self, parameters):
        """
        Initialize planner with weld parameters.

        Args:
            parameters : dict
                Dictionary with keys:
                - joint_type: Type of weld joint
                - work_angle_deg: Work angle in degrees
                - travel_angle_deg: Travel angle in degrees
                - gap_mm: Gap distance in millimeters
                - num_points: Number of poses to generate per seam

        """
        self.joint_type = parameters['joint_type'].lower()
        self.work_angle_rad = np.radians(parameters['work_angle_deg'])
        self.travel_angle_rad = np.radians(parameters['travel_angle_deg'])
        self.gap_m = parameters['gap_mm'] / 1000.0
        self.num_points = parameters['num_points']

        if self.joint_type not in self.SUPPORTED_JOINT_TYPES:
            raise ValueError(f"Unsupported joint type: {self.joint_type}")
        if not (1 <= self.num_points <= 1000):
            raise ValueError(f"num_points must be between 1 and 1000, got {self.num_points}")
        if self.gap_m <= 0:
            raise ValueError(f"gap_mm must be positive, got {parameters['gap_mm']}")

    def generate_seam(self, seam, surface_info, lean_sign=1):
        """
        Generate poses for a seam.

        Modifies seam object in-place by setting seam.poses and seam.is_generated.

        Args:
            seam : Seam
                Seam object to process
            surface_info : dict
                Dictionary with 'center' and 'normal' keys
            lean_sign : int, optional
                Sign for lean/radial direction (+1 or -1), used for lap_joint
                and corner_joint variants (default is 1)

        Returns
        -------
        bool
            True if successful, False otherwise

        """
        try:
            surface_center = np.array(surface_info['center'], dtype=float)
            surface_normal = np.array(surface_info['normal'], dtype=float)
            surface_normal = surface_normal / np.linalg.norm(surface_normal)

            line = seam.line_segment

            self._validate_seam(line, surface_center, surface_normal)

            tangent = line.tangent()
            radial = self._calculate_radial_direction(
                line.start, line.end, surface_center, surface_normal
            )

            main_direction = self._get_main_direction(surface_normal, radial)
            lean_direction = self._get_lean_direction(surface_normal, radial, lean_sign)

            tangent_base, binormal_base, normal_base = self._build_base_frame(tangent, main_direction)

            normal_work, binormal_work, tangent_work = self._apply_work_angle(
                normal_base, tangent_base, binormal_base, lean_direction
            )

            normal_final, binormal_final, tangent_final = self._apply_travel_angle(
                normal_work, binormal_work, tangent_work
            )

            gap_offset = self._calculate_gap_offset(surface_normal, radial, lean_sign)

            poses = []
            for i in range(self.num_points):
                t = i / max(1, self.num_points - 1)
                position = line.point_at(t) + gap_offset

                pose = self._build_pose_data(
                    position, tangent_final, binormal_final, normal_final, i
                )
                poses.append(pose)

            seam.poses = poses
            seam.is_generated = True

            return True

        except Exception as e:
            print(f"Error generating seam: {e}")
            seam.is_generated = False
            return False

    def requires_variants(self):
        """
        Check if this joint type requires generating both +/- variants.

        Returns
        -------
        bool
            True if lap_joint or corner_joint

        """
        return self.joint_type in ["lap_joint", "corner_joint"]

    def _validate_seam(self, line, surface_center, surface_normal):
        """Validate that seam geometry is reasonable."""
        if line.length() < 1e-6:
            raise ValueError("Seam is too short (degenerate)")

        if line.length() > 10.0:
            raise ValueError(f"Seam is too long ({line.length():.2f}m > 10m)")

        tolerance = 0.01  # 10mm

        dist_start = abs(np.dot(line.start - surface_center, surface_normal))
        dist_end = abs(np.dot(line.end - surface_center, surface_normal))

        if dist_start > tolerance:
            raise ValueError(f"Seam start is {dist_start*1000:.1f}mm from surface plane")
        if dist_end > tolerance:
            raise ValueError(f"Seam end is {dist_end*1000:.1f}mm from surface plane")

    def _calculate_radial_direction(
        self,
        seam_start: np.ndarray,
        seam_end: np.ndarray,
        surface_center: np.ndarray,
        surface_normal: np.ndarray,
        min_offset_m: float = 0.01,
    ) -> np.ndarray:
        """
        Calculate radial direction: perpendicular from infinite seam line to surface center.

        Args:
            seam_start : np.ndarray
                Start point of seam
            seam_end : np.ndarray
                End point of seam
            surface_center : np.ndarray
                Center point of surface
            surface_normal : np.ndarray
                Normal vector of surface
            min_offset_m : float, optional
                Minimum perpendicular distance required (default 10mm)

        Returns
        -------
        np.ndarray
            Normalized radial direction vector (from line toward center)

        Raises
        ------
        ValueError
            If center is too close to seam line

        """
        tangent = seam_end - seam_start
        tangent = tangent / np.linalg.norm(tangent)

        vec_to_center = surface_center - seam_start
        t = np.dot(vec_to_center, tangent)
        closest_point_on_line = seam_start + t * tangent

        from_line_to_center = surface_center - closest_point_on_line

        from_line_to_center_in_plane = from_line_to_center - np.dot(
            from_line_to_center, surface_normal
        ) * surface_normal

        distance = np.linalg.norm(from_line_to_center_in_plane)
        if distance < min_offset_m:
            raise ValueError(
                f"Seam line too close to surface center "
                f"(perpendicular distance: {distance*1000:.1f}mm, minimum: {min_offset_m*1000:.1f}mm). "
                f"Cannot determine radial direction."
            )

        radial_direction = from_line_to_center_in_plane / distance

        return radial_direction

    def _get_main_direction(self, surface_normal: np.ndarray, radial_direction: np.ndarray) -> np.ndarray:
        """
        Calculate main torch direction (base orientation before work angle).

        Args:
            surface_normal : np.ndarray
                Normal vector of surface
            radial_direction : np.ndarray
                Radial direction from seam line to center

        Returns
        -------
        np.ndarray
            Main direction vector (where torch points by default)

        """
        if self.joint_type in ["t_joint", "butt_joint", "lap_joint", "corner_joint"]:
            return -surface_normal
        elif self.joint_type == "edge_joint":
            return -radial_direction
        else:
            raise ValueError(f"Unknown joint type: {self.joint_type}")

    def _get_lean_direction(self, surface_normal: np.ndarray, radial_direction: np.ndarray, lean_sign: int = 1) -> np.ndarray:
        """
        Calculate lean direction (direction to apply work angle tilt).

        Args:
            surface_normal : np.ndarray
                Normal vector of surface
            radial_direction : np.ndarray
                Radial direction from seam line to center
            lean_sign : int, optional
                Sign multiplier for radial direction (+1 or -1), used for lap/corner
                variants (default is 1)

        Returns
        -------
        np.ndarray
            Lean direction vector (which way to tilt torch)

        """
        if self.joint_type in ["t_joint", "butt_joint"]:
            return radial_direction
        elif self.joint_type == "edge_joint":
            return -surface_normal
        elif self.joint_type in ["lap_joint", "corner_joint"]:
            return lean_sign * radial_direction
        else:
            raise ValueError(f"Unknown joint type: {self.joint_type}")

    def _build_base_frame(
        self,
        tangent: np.ndarray,
        main_direction: np.ndarray,
    ) -> tuple:
        """
        Build orthonormal frame from tangent and main direction.

        Args:
            tangent : np.ndarray
                Direction along seam (travel direction)
            main_direction : np.ndarray
                Main torch direction

        Returns
        -------
        tuple
            Tuple of (tangent, binormal, normal) forming right-handed
            orthonormal frame

        """
        normal = main_direction / np.linalg.norm(main_direction)
        binormal = np.cross(normal, tangent)
        binormal = binormal / np.linalg.norm(binormal)

        tangent = np.cross(binormal, normal)
        tangent = tangent / np.linalg.norm(tangent)

        return tangent, binormal, normal

    def _apply_work_angle(
        self,
        normal: np.ndarray,
        tangent: np.ndarray,
        binormal: np.ndarray,
        lean_direction: np.ndarray,
    ) -> tuple:
        """
        Apply work angle rotation, choosing direction that tilts toward lean direction.

        Args:
            normal : np.ndarray
                Normal vector (torch direction)
            tangent : np.ndarray
                Tangent vector (travel direction)
            binormal : np.ndarray
                Binormal vector
            lean_direction : np.ndarray
                Target direction for tilting

        Returns
        -------
        tuple
            Tuple of rotated (normal, binormal, tangent)

        """
        work_rot_positive = Rotation.from_rotvec(self.work_angle_rad * tangent)
        normal_test_pos = work_rot_positive.apply(normal)
        dot_positive = np.dot(normal_test_pos, lean_direction)

        work_rot_negative = Rotation.from_rotvec(-self.work_angle_rad * tangent)
        normal_test_neg = work_rot_negative.apply(normal)
        dot_negative = np.dot(normal_test_neg, lean_direction)

        if dot_positive > dot_negative:
            work_rot = work_rot_positive
        else:
            work_rot = work_rot_negative

        normal_rotated = work_rot.apply(normal)
        binormal_rotated = work_rot.apply(binormal)
        tangent_rotated = tangent

        return normal_rotated, binormal_rotated, tangent_rotated

    def _apply_travel_angle(
        self,
        normal: np.ndarray,
        binormal: np.ndarray,
        tangent: np.ndarray,
    ) -> tuple:
        """
        Apply travel angle rotation around binormal axis.

        Args:
            normal : np.ndarray
                Normal vector
            binormal : np.ndarray
                Binormal vector
            tangent : np.ndarray
                Tangent vector

        Returns
        -------
        tuple
            Tuple of rotated (normal, binormal, tangent)

        """
        travel_rot = Rotation.from_rotvec(self.travel_angle_rad * binormal)
        tangent_final = travel_rot.apply(tangent)
        binormal_final = travel_rot.apply(binormal)
        normal_final = travel_rot.apply(normal)

        return normal_final, binormal_final, tangent_final

    def _calculate_gap_offset(
        self,
        surface_normal: np.ndarray,
        radial_direction: np.ndarray,
        lean_sign: int = 1,
    ) -> np.ndarray:
        """
        Calculate gap offset from seam based on joint type.

        Args:
            surface_normal : np.ndarray
                Normal vector of surface
            radial_direction : np.ndarray
                Radial direction from seam line to center
            lean_sign : int, optional
                Sign multiplier for radial direction (+1 or -1), used for lap/corner
                variants (default is 1)

        Returns
        -------
        np.ndarray
            Gap offset vector

        """
        gap_component = self.gap_m / np.sqrt(2.0)

        if self.joint_type in ["t_joint", "butt_joint"]:
            return surface_normal * gap_component + radial_direction * gap_component

        elif self.joint_type == "edge_joint":
            return -surface_normal * gap_component + (-radial_direction) * gap_component

        elif self.joint_type in ["lap_joint", "corner_joint"]:
            return surface_normal * gap_component + (lean_sign * radial_direction) * gap_component

        else:
            raise ValueError(f"Unknown joint type: {self.joint_type}")

    def _build_pose_data(
        self,
        position: np.ndarray,
        tangent: np.ndarray,
        binormal: np.ndarray,
        normal: np.ndarray,
        index: int,
    ) -> dict:
        """
        Build pose data dictionary with position, quaternion, and transformation matrix.

        Args:
            position : np.ndarray
                3D position vector
            tangent : np.ndarray
                Tangent vector (X-axis in ROS2 convention)
            binormal : np.ndarray
                Binormal vector (Y-axis will be -binormal)
            normal : np.ndarray
                Normal vector (Z-axis will be -normal)
            index : int
                Point index

        Returns
        -------
        dict
            Dictionary with pose data

        """
        rot_matrix = np.column_stack([tangent, -binormal, -normal])

        quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rot_matrix
        transform_matrix[:3, 3] = position

        return {
            "index": index,
            "position": position.tolist(),
            "quaternion": quat.tolist(),
            "matrix": transform_matrix.tolist(),
        }
