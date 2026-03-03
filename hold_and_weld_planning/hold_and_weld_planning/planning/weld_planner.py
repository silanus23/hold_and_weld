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

This module generates weld torch poses along seam paths using dual surface normals
from both meshes. It handles coordinate frame construction, work/travel angle
application, and gap offset calculations.

The planner uses normals pre-computed by SeamExtractor to determine torch orientation
and gap positioning without needing manual specification of away_from_wall vectors.

All seams generate dense waypoint paths with configurable spacing (default 10mm) to
ensure proper torch orientation throughout the path, even for nominally straight seams
where surface normals may vary.
"""

from typing import Any, Dict, List

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


class WeldPlanner:
    """Generate weld paths for seams using dual surface normals.

    Modifies seam objects in-place following the Mutable State pattern.
    Uses normals from both meshes (pre-classified as main/secondary by
    SeamExtractor) to compute torch orientation and gap offset.

    The planner applies work angle (torch tilt toward lean direction) and
    travel angle (torch tilt along travel direction) to generate final poses.
    """

    def __init__(self, parameters: Dict[str, Any]) -> None:
        """Initialize planner with weld parameters.

        Args:
            parameters: Dictionary with keys:
                - work_angle_deg: Work angle in degrees (torch tilt perpendicular to travel)
                - travel_angle_deg: Travel angle in degrees (torch tilt along travel)
                - gap_mm: Gap distance from seam in millimeters
                - waypoint_spacing_mm: Distance between waypoints (default 10mm)
                - up_vector: World up vector [x, y, z] (default [0, 0, 1])

        Raises:
            ValueError: If gap_mm or waypoint_spacing_mm is non-positive
        """
        self.work_angle_rad = np.radians(parameters['work_angle_deg'])
        self.travel_angle_rad = np.radians(parameters['travel_angle_deg'])
        self.gap_m = parameters['gap_mm'] / 1000.0
        self.waypoint_spacing_m = parameters.get('waypoint_spacing_mm', 10.0) / 1000.0
        self.up_vector = np.array(
            parameters.get('up_vector', [0.0, 0.0, 1.0]), dtype=float
        )
        self.up_vector = self.up_vector / np.linalg.norm(self.up_vector)

        if self.gap_m <= 0:
            raise ValueError(f'gap_mm must be positive, got {parameters["gap_mm"]}')

        if self.waypoint_spacing_m <= 0:
            raise ValueError(
                f'waypoint_spacing_mm must be positive, '
                f'got {parameters.get("waypoint_spacing_mm", 10.0)}'
            )

    def generate_seam(self, seam: Any) -> None:
        """Generate dense waypoint path for seam. Modifies seam object in place.

        All geometry types (line/arc/polyline) generate dense waypoints with
        configurable spacing to ensure proper torch orientation throughout,
        even where surface normals vary along nominally straight seams.

        Args:
            seam: Seam object with geometry data in config

        Raises:
            RuntimeError: If required data missing from seam.config
            ValueError: If arrays have invalid lengths

        Side Effects:
            - Sets seam.poses to list of pose dictionaries
            - Sets seam.is_generated to True
        """
        if 'smoothed_points' not in seam.config:
            raise RuntimeError('Seam missing smoothed_points in config')
        if 'normals_main' not in seam.config:
            raise RuntimeError('Seam missing normals_main in config')
        if 'normals_secondary' not in seam.config:
            raise RuntimeError('Seam missing normals_secondary in config')
        if 'is_edge_joint' not in seam.config:
            raise RuntimeError('Seam missing is_edge_joint in config')

        points = seam.config['smoothed_points']
        normals_main = seam.config['normals_main']
        normals_secondary = seam.config['normals_secondary']
        is_edge_joint = seam.config['is_edge_joint']

        self._validate_arrays(points, normals_main, normals_secondary)

        sampled_indices = self._sample_by_distance(points, self.waypoint_spacing_m)

        poses = []
        for idx in sampled_indices:
            pose = self._compute_pose_at_index(
                points, normals_main, normals_secondary, is_edge_joint, idx
            )
            poses.append(pose)

        seam.poses = poses
        seam.is_generated = True

    def _validate_arrays(
        self, points: NDArray, normals_main: NDArray, normals_secondary: NDArray
    ) -> None:
        """Validate that input arrays have consistent lengths.

        Args:
            points: Seam points array
            normals_main: Main normals array
            normals_secondary: Secondary normals array

        Raises:
            ValueError: If arrays have mismatched lengths or too few points
        """
        if len(points) < 2:
            raise ValueError('Need at least 2 points to generate poses')

        if len(normals_main) != len(points):
            raise ValueError(
                f'normals_main length {len(normals_main)} does not match '
                f'points length {len(points)}'
            )

        if len(normals_secondary) != len(points):
            raise ValueError(
                f'normals_secondary length {len(normals_secondary)} does not match '
                f'points length {len(points)}'
            )

    def _sample_by_distance(self, points: NDArray, spacing: float) -> List[int]:
        """Sample point indices based on distance threshold.

        Args:
            points: Array of points (N, 3)
            spacing: Minimum distance between samples in meters

        Returns:
            List of indices into points array
        """
        if len(points) == 0:
            return []

        sampled = [0]
        cumulative_dist = 0.0

        for i in range(1, len(points)):
            segment_dist = np.linalg.norm(points[i] - points[i-1])
            cumulative_dist += segment_dist

            if cumulative_dist >= spacing:
                sampled.append(i)
                cumulative_dist = 0.0

        if sampled[-1] != len(points) - 1:
            sampled.append(len(points) - 1)

        return sampled

    def _compute_pose_at_index(
        self,
        points: NDArray,
        normals_main: NDArray,
        normals_secondary: NDArray,
        is_edge_joint: bool,
        index: int,
    ) -> Dict[str, Any]:
        """Compute torch pose at specific point index.

        Args:
            points: Array of seam points (N, 3)
            normals_main: Main surface normals (N, 3)
            normals_secondary: Secondary surface normals (N, 3)
            is_edge_joint: Whether this is edge-to-edge joint
            index: Index of point to compute pose for

        Returns:
            Pose dictionary with position, quaternion, and matrix
        """
        tangent = self._compute_tangent(points, index)
        main_normal = normals_main[index]
        secondary_normal = normals_secondary[index]

        away_from_wall = self._compute_away_vector(
            tangent, main_normal, secondary_normal
        )

        if is_edge_joint:
            main_direction = away_from_wall
            lean_direction = -main_normal
            gap_offset_direction = main_normal
            gap_magnitude = self.gap_m
        else:
            main_direction = -main_normal
            lean_direction = away_from_wall
            gap_offset_direction = -away_from_wall + main_normal
            gap_magnitude = self.gap_m

        # Normalize gap offset direction to ensure correct offset distance
        gap_offset_direction_norm = np.linalg.norm(gap_offset_direction)
        if gap_offset_direction_norm > 1e-10:
            gap_offset_direction = (gap_offset_direction / gap_offset_direction_norm)
        else:
            # Fallback: use main_normal if direction is degenerate
            gap_offset_direction = (main_normal / np.linalg.norm(main_normal))

        tangent_base, binormal_base, normal_base = self._build_base_frame(
            tangent, main_direction
        )

        normal_work, binormal_work, tangent_work = self._apply_work_angle(
            normal_base, tangent_base, binormal_base, lean_direction
        )

        normal_final, binormal_final, tangent_final = self._apply_travel_angle(
            normal_work, binormal_work, tangent_work
        )

        gap_offset = gap_magnitude * gap_offset_direction
        position = points[index] + gap_offset

        pose = self._build_pose_data(
            position, tangent_final, binormal_final, normal_final, index
        )

        return pose

    def _compute_tangent(self, points: NDArray, index: int) -> NDArray:
        """Compute tangent vector at point index along seam path.

        Uses central differences for interior points, forward/backward
        differences for endpoints.

        Args:
            points: Array of seam points (N, 3)
            index: Index of point where tangent is needed

        Returns:
            Normalized tangent vector (3,) pointing in travel direction
        """
        if index == 0:
            tangent = points[1] - points[0]
        elif index == len(points) - 1:
            tangent = points[-1] - points[-2]
        else:
            tangent = points[index + 1] - points[index - 1]

        norm = np.linalg.norm(tangent)
        if norm < 1e-10:
            return np.array([1, 0, 0])

        return tangent / norm

    def _compute_away_vector(
        self, tangent: NDArray, main_normal: NDArray, secondary_normal: NDArray
    ) -> NDArray:
        """Compute away_from_wall vector perpendicular to seam.

        Computes a vector perpendicular to both the tangent and main normal,
        then checks its direction relative to secondary normal to ensure it
        points away from the secondary piece (edge piece).

        Args:
            tangent: Normalized tangent vector along seam (3,)
            main_normal: Normal from main (base/flat) surface (3,)
            secondary_normal: Normal from secondary (edge) piece (3,)

        Returns:
            Normalized vector (3,) perpendicular to seam, pointing away
            from secondary piece
        """
        perpendicular = np.cross(main_normal, tangent)
        norm = np.linalg.norm(perpendicular)

        if norm < 1e-10:
            perpendicular = np.cross(np.array([0, 0, 1]), tangent)
            norm = np.linalg.norm(perpendicular)
            if norm < 1e-10:
                perpendicular = np.cross(np.array([1, 0, 0]), tangent)
                norm = np.linalg.norm(perpendicular)

        perpendicular = perpendicular / np.linalg.norm(perpendicular)

        if np.dot(perpendicular, secondary_normal) > 0:
            return -perpendicular
        else:
            return perpendicular

    def _build_base_frame(
        self,
        tangent: NDArray,
        main_direction: NDArray,
    ) -> tuple[NDArray, NDArray, NDArray]:
        """Build orthonormal coordinate frame for torch.

        Constructs right-handed coordinate system with:
        - Normal (Z-axis): torch pointing direction
        - Tangent (X-axis): travel direction
        - Binormal (Y-axis): perpendicular to both

        Recomputes tangent from binormal × normal to enforce strict orthogonality.

        Args:
            tangent: Approximate travel direction (3,)
            main_direction: Desired torch pointing direction (3,)

        Returns:
            Tuple of (tangent, binormal, normal), each normalized (3,)
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
    ) -> tuple[NDArray, NDArray, NDArray]:
        """Apply work angle rotation toward lean direction.

        Rotates torch around tangent (travel) axis by work_angle_rad toward
        the lean direction. For edge-to-surface joints, this tilts the torch
        away from the vertical edge piece. For edge-to-edge joints, this
        tilts inward along the main normal.

        The rotation sign is determined by checking which side of the tangent
        axis the lean_direction falls on.

        Args:
            normal: Current torch direction (3,)
            tangent: Travel direction (3,)
            binormal: Cross product of normal and tangent (3,)
            lean_direction: Target direction for tilting (3,)

        Returns:
            Tuple of rotated (normal, binormal, tangent), each (3,)
        """
        sign = np.sign(np.dot(np.cross(normal, lean_direction), tangent))
        work_rot = Rotation.from_rotvec(sign * self.work_angle_rad * tangent)

        normal_rotated = work_rot.apply(normal)
        binormal_rotated = work_rot.apply(binormal)
        tangent_rotated = tangent

        return normal_rotated, binormal_rotated, tangent_rotated

    def _apply_travel_angle(
        self,
        normal: NDArray,
        binormal: NDArray,
        tangent: NDArray,
    ) -> tuple[NDArray, NDArray, NDArray]:
        """Apply travel angle rotation around binormal axis.

        Rotates torch around binormal (perpendicular to travel) axis by
        travel_angle_rad. This tilts the torch forward or backward along
        the direction of travel.

        Args:
            normal: Current torch direction after work angle (3,)
            binormal: Perpendicular to travel direction (3,)
            tangent: Travel direction (3,)

        Returns:
            Tuple of rotated (normal, binormal, tangent), each (3,)
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

        Constructs final pose as:
        - 3D position (center of torch)
        - Quaternion orientation
        - 4x4 transformation matrix

        The coordinate frame convention is:
        - X-axis (tangent): travel direction
        - Y-axis (binormal): perpendicular to travel
        - Z-axis (normal): torch pointing direction

        Args:
            position: 3D position of torch center (3,)
            tangent: Final travel direction after angles applied (3,)
            binormal: Final perpendicular direction (3,)
            normal: Final torch pointing direction (3,)
            index: Sequential point index along seam

        Returns:
            Dictionary with keys:
                - index: int point index
                - position: list[float] of [x, y, z]
                - quaternion: list[float] of [x, y, z, w]
                - matrix: list[list[float]] 4x4 transformation matrix
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
