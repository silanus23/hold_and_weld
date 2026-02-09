# Copyright 2025 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""Weld path planner - handles all geometric calculations for trajectory generation.

This module generates weld torch poses along seam paths based on joint type and
welding parameters. It handles coordinate frame construction, work/travel angle
application, and gap offset calculations for various joint configurations.
"""

from typing import Any, Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


class WeldPlanner:
    """Generate weld poses for seams based on joint type and weld parameters.

    Modifies seam objects in-place following the Mutable State pattern.
    """

    SUPPORTED_JOINT_TYPES = [
        't_joint', 'butt_joint', 'corner_joint', 'lap_joint', 'edge_joint'
    ]

    def __init__(self, parameters: Dict[str, Any]) -> None:
        """Initialize planner with weld parameters.

        Args:
            parameters: Dictionary with keys:
                - joint_type: Type of weld joint (str)
                - work_angle_deg: Work angle in degrees (float)
                - travel_angle_deg: Travel angle in degrees (float)
                - gap_mm: Gap distance in millimeters (float)
                - num_points: Number of poses to generate per seam (int)
                
        Raises:
            ValueError: If joint_type unsupported, num_points out of range,
                or gap_mm is non-positive.
        """
        self.joint_type = parameters['joint_type'].lower()
        self.work_angle_rad = np.radians(parameters['work_angle_deg'])
        self.travel_angle_rad = np.radians(parameters['travel_angle_deg'])
        self.gap_m = parameters['gap_mm'] / 1000.0
        self.num_points = parameters['num_points']

        if self.joint_type not in self.SUPPORTED_JOINT_TYPES:
            raise ValueError(f'Unsupported joint type: {self.joint_type}')
        if not (1 <= self.num_points <= 1000):
            raise ValueError(
                f'num_points must be between 1 and 1000, '
                f'got {self.num_points}'
            )
        if self.gap_m <= 0:
            raise ValueError(
                f"gap_mm must be positive, got {parameters['gap_mm']}"
            )

    def generate_seam(
        self,
        seam: Any,
        surface_info: Dict[str, List[float]],
        away_from_wall_vector: List[float] | NDArray | None = None,
        lean_sign: int = 1
    ) -> bool:
        """Generate poses for a seam. Modifies seam object in place.

        Args:
            seam: Seam object with line_segment attribute to process.
            surface_info: Dictionary with 'center' and 'normal' keys (lists of floats).
            away_from_wall_vector: Vector pointing away from wall/material. If None,
                raises error (manual users must provide this).
            lean_sign: Sign for lean/away direction (+1 or -1), used for
                lap_joint and corner_joint variants (default 1).

        Returns:
            True if successful, False if error occurred.
            
        Raises:
            ValueError: If away_from_wall_vector is None (required for all seams).
        """
        try:
            if away_from_wall_vector is None:
                raise ValueError(
                    "away_from_wall_vector is required. "
                    "For URDF workflow, this is auto-calculated. "
                    "For manual workflow, you must specify it in your seam config."
                )
            
            surface_center = np.array(surface_info['center'], dtype=float)
            surface_normal = np.array(surface_info['normal'], dtype=float)
            surface_normal = surface_normal / np.linalg.norm(surface_normal)

            line = seam.line_segment

            self._validate_seam(line, surface_center, surface_normal)

            tangent = line.tangent()
            
            # Use the provided away_from_wall_vector
            away_vec = np.array(away_from_wall_vector, dtype=float)
            away_vec = away_vec / np.linalg.norm(away_vec)  # Normalize

            main_direction = self._get_main_direction(
                surface_normal, away_vec
            )
            lean_direction = self._get_lean_direction(
                surface_normal, away_vec, lean_sign
            )

            tangent_base, binormal_base, normal_base = (
                self._build_base_frame(tangent, main_direction)
            )

            normal_work, binormal_work, tangent_work = (
                self._apply_work_angle(
                    normal_base, tangent_base, binormal_base, lean_direction
                )
            )

            normal_final, binormal_final, tangent_final = (
                self._apply_travel_angle(
                    normal_work, binormal_work, tangent_work
                )
            )

            gap_offset = self._calculate_gap_offset(
                surface_normal, away_vec, lean_sign
            )

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
            print(f'Error generating seam: {e}')
            seam.is_generated = False
            return False

    def requires_variants(self) -> bool:
        """Check if this joint type requires generating both +/- lean variants.

        Returns:
            True if lap_joint or corner_joint, False otherwise.
        """
        return self.joint_type in ['lap_joint', 'corner_joint']

    def _validate_seam(
        self,
        line: Any,
        surface_center: NDArray,
        surface_normal: NDArray
    ) -> None:
        """Validate seam geometry and position relative to surface.
        
        Args:
            line: LineSegment object to validate.
            surface_center: Center point of surface.
            surface_normal: Normal vector of surface.
            
        Raises:
            ValueError: If seam is degenerate, too long, or not on surface plane.
        """
        if line.length() < 1e-6:
            raise ValueError('Seam is too short (degenerate)')

        if line.length() > 10.0:
            raise ValueError(
                f'Seam is too long ({line.length():.2f}m > 10m)'
            )

        tolerance = 0.01

        dist_start = abs(np.dot(line.start - surface_center, surface_normal))
        dist_end = abs(np.dot(line.end - surface_center, surface_normal))

        if dist_start > tolerance:
            raise ValueError(
                f'Seam start is {dist_start*1000:.1f}mm from surface plane'
            )
        if dist_end > tolerance:
            raise ValueError(
                f'Seam end is {dist_end*1000:.1f}mm from surface plane'
            )

    def _get_main_direction(
        self, surface_normal: NDArray, away_from_wall_vector: NDArray
    ) -> NDArray:
        """Calculate main torch direction (base orientation before work angle).

        Args:
            surface_normal: Normal vector of surface.
            away_from_wall_vector: Vector pointing away from wall/material.

        Returns:
            Main direction vector (where torch points by default).
            
        Raises:
            ValueError: If joint_type is unknown.
        """
        if self.joint_type in [
            't_joint', 'butt_joint', 'lap_joint', 'corner_joint'
        ]:
            return -surface_normal
        elif self.joint_type == 'edge_joint':
            return -away_from_wall_vector
        else:
            raise ValueError(f'Unknown joint type: {self.joint_type}')

    def _get_lean_direction(
        self,
        surface_normal: NDArray,
        away_from_wall_vector: NDArray,
        lean_sign: int = 1
    ) -> NDArray:
        """Calculate lean direction (direction to apply work angle tilt).

        Args:
            surface_normal: Normal vector of surface.
            away_from_wall_vector: Vector pointing away from wall/material.
            lean_sign: Sign multiplier for away direction (+1 or -1), used
                for lap/corner variants (default 1).

        Returns:
            Lean direction vector (which way to tilt torch).
            
        Raises:
            ValueError: If joint_type is unknown.
        """
        if self.joint_type in ['t_joint', 'butt_joint']:
            return away_from_wall_vector
        elif self.joint_type == 'edge_joint':
            return -surface_normal
        elif self.joint_type in ['lap_joint', 'corner_joint']:
            return lean_sign * away_from_wall_vector
        else:
            raise ValueError(f'Unknown joint type: {self.joint_type}')

    def _build_base_frame(
        self,
        tangent: NDArray,
        main_direction: NDArray,
    ) -> Tuple[NDArray, NDArray, NDArray]:
        """Build orthonormal frame from tangent and main direction.

        Args:
            tangent: Direction along seam (travel direction).
            main_direction: Main torch direction.

        Returns:
            Tuple of (tangent, binormal, normal) forming right-handed
            orthonormal frame.
        """
        normal = main_direction / np.linalg.norm(main_direction)
        binormal = np.cross(normal, tangent)
        binormal = binormal / np.linalg.norm(binormal)

        tangent = np.cross(binormal, normal)
        tangent = tangent / np.linalg.norm(tangent)

        return tangent, binormal, normal

    def _apply_work_angle(
        self,
        normal: NDArray,
        tangent: NDArray,
        binormal: NDArray,
        lean_direction: NDArray,
    ) -> Tuple[NDArray, NDArray, NDArray]:
        """Apply work angle rotation, choosing direction that tilts toward lean.

        Args:
            normal: Normal vector (torch direction).
            tangent: Tangent vector (travel direction).
            binormal: Binormal vector.
            lean_direction: Target direction for tilting.

        Returns:
            Tuple of rotated (normal, binormal, tangent).
        """
        work_rot_positive = Rotation.from_rotvec(
            self.work_angle_rad * tangent
        )
        normal_test_pos = work_rot_positive.apply(normal)
        dot_positive = np.dot(normal_test_pos, lean_direction)

        work_rot_negative = Rotation.from_rotvec(
            -self.work_angle_rad * tangent
        )
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
        normal: NDArray,
        binormal: NDArray,
        tangent: NDArray,
    ) -> Tuple[NDArray, NDArray, NDArray]:
        """Apply travel angle rotation around binormal axis.

        Args:
            normal: Normal vector.
            binormal: Binormal vector.
            tangent: Tangent vector.

        Returns:
            Tuple of rotated (normal, binormal, tangent).
        """
        travel_rot = Rotation.from_rotvec(self.travel_angle_rad * binormal)
        tangent_final = travel_rot.apply(tangent)
        binormal_final = travel_rot.apply(binormal)
        normal_final = travel_rot.apply(normal)

        return normal_final, binormal_final, tangent_final

    def _calculate_gap_offset(
        self,
        surface_normal: NDArray,
        away_from_wall_vector: NDArray,
        lean_sign: int = 1,
    ) -> NDArray:
        """Calculate gap offset from seam based on joint type.

        Args:
            surface_normal: Normal vector of surface.
            away_from_wall_vector: Vector pointing away from wall/material.
            lean_sign: Sign multiplier for away direction (+1 or -1), used
                for lap/corner variants (default 1).

        Returns:
            Gap offset vector.
            
        Raises:
            ValueError: If joint_type is unknown.
        """
        gap_component = self.gap_m / np.sqrt(2.0)

        if self.joint_type in ['t_joint', 'butt_joint']:
            return (
                surface_normal * gap_component -
                away_from_wall_vector * gap_component
            )

        elif self.joint_type == 'edge_joint':
            return (
                -surface_normal * gap_component +
                (-away_from_wall_vector) * gap_component
            )

        elif self.joint_type in ['lap_joint', 'corner_joint']:
            return (
                surface_normal * gap_component +
                (lean_sign * away_from_wall_vector) * gap_component
            )

        else:
            raise ValueError(f'Unknown joint type: {self.joint_type}')

    def _build_pose_data(
        self,
        position: NDArray,
        tangent: NDArray,
        binormal: NDArray,
        normal: NDArray,
        index: int,
    ) -> Dict[str, Any]:
        """Build pose data dictionary with position and orientation.

        Args:
            position: 3D position vector.
            tangent: Tangent vector (X-axis).
            binormal: Binormal vector (Y-axis will be -binormal).
            normal: Normal vector (Z-axis will be -normal).
            index: Point index in sequence.

        Returns:
            Dictionary with 'index', 'position', 'quaternion', and 'matrix' keys.
        """
        rot_matrix = np.column_stack([tangent, -binormal, -normal])

        quat = Rotation.from_matrix(rot_matrix).as_quat()

        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rot_matrix
        transform_matrix[:3, 3] = position

        return {
            'index': index,
            'position': position.tolist(),
            'quaternion': quat.tolist(),
            'matrix': transform_matrix.tolist(),
        }
