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

This module generates weld torch poses along seam paths using surface normal and
away_from_wall_vector geometry. It handles coordinate frame construction, work/travel
angle application, and gap offset calculations.
"""

from typing import Any, Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


class WeldPlanner:
    """Generate weld poses for seams using surface geometry vectors.

    Modifies seam objects in-place following the Mutable State pattern.
    Uses surface_normal and away_from_wall_vector directly without
    joint type classification for torch orientation.
    """

    def __init__(self, parameters: Dict[str, Any]) -> None:
        """Initialize planner with weld parameters.

        Args:
            parameters: Dictionary with keys:
                - work_angle_deg: Work angle in degrees (float)
                - travel_angle_deg: Travel angle in degrees (float)
                - gap_mm: Gap distance in millimeters (float)
                - num_points: Number of poses to generate per seam (int)

        Raises:
            ValueError: If num_points out of range or gap_mm is non-positive.
        """
        self.work_angle_rad = np.radians(parameters['work_angle_deg'])
        self.travel_angle_rad = np.radians(parameters['travel_angle_deg'])
        self.gap_m = parameters['gap_mm'] / 1000.0
        self.num_points = parameters['num_points']

        if not (1 <= self.num_points <= 1000):
            raise ValueError(
                f'num_points must be between 1 and 1000, got {self.num_points}'
            )
        if self.gap_m <= 0:
            raise ValueError(
                f"gap_mm must be positive, got {parameters['gap_mm']}"
            )

    def generate_seam(
        self,
        seam: Any,
        surface_info: Dict[str, List[float]]
    ) -> None:
        """Generate poses for a seam. Modifies seam object in place.

        Args:
            seam: Seam object with line_segment attribute containing away_from_wall_vector.
            surface_info: Dictionary with 'center' and 'normal' keys (lists of floats).

        Raises:
            RuntimeError: If away_from_wall_vector not set in seam.line_segment.
            ValueError: If seam geometry is invalid or not on surface plane.
        """
        if seam.line_segment.away_from_wall_vector is None:
            raise RuntimeError(
                f'away_from_wall_vector not set for seam {seam}. '
                'For manual workflow: specify it in YAML config. '
                'For URDF workflow: ensure auto-detection calculates it.'
            )

        surface_center = np.array(surface_info['center'], dtype=float)
        surface_normal = np.array(surface_info['normal'], dtype=float)
        surface_normal = surface_normal / np.linalg.norm(surface_normal)

        line = seam.line_segment
        self._validate_seam(line, surface_center, surface_normal)

        tangent = line.tangent()
        away_from_wall_vector = line.away_from_wall_vector
        is_edge_joint = seam.config.get('is_edge_joint', False)

        # Calculate directions based on joint type
        # Directions point WHERE the torch should point (toward the work)
        if is_edge_joint:
            # Edge joint: point toward material, lean toward surface
            main_direction = -away_from_wall_vector
            lean_direction = -surface_normal
        else:
            # Flat joint: point toward surface, lean toward wall
            main_direction = -surface_normal
            lean_direction = -away_from_wall_vector

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

        if is_edge_joint:
            # Edge joint: offset along bisector (normal + away direction)
            gap_component = self.gap_m / np.sqrt(2.0)
            gap_offset = gap_component * (-surface_normal + away_from_wall_vector)
        else:
            # Flat joint: offset perpendicular to surface and away from other part
            gap_component = self.gap_m / np.sqrt(2.0)
            gap_offset = gap_component * (-surface_normal - away_from_wall_vector)

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
                f'Seam start is {dist_start*1000:.1f}mm from surface plane '
                f'(tolerance: {tolerance*1000:.1f}mm)'
            )
        if dist_end > tolerance:
            raise ValueError(
                f'Seam end is {dist_end*1000:.1f}mm from surface plane '
                f'(tolerance: {tolerance*1000:.1f}mm)'
            )

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
            binormal: Binormal vector (Y-axis).
            normal: Normal vector (Z-axis - torch pointing direction).
            index: Point index in sequence.

        Returns:
            Dictionary with 'index', 'position', 'quaternion', and 'matrix' keys.
        """
        rot_matrix = np.column_stack([tangent, binormal, normal])

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
