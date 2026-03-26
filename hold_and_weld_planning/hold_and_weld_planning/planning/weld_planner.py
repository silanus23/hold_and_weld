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

"""Generate weld torch poses along seam paths using dual surface normals."""

from typing import Any, Dict, List

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


class WeldPlanner:
    """Generate weld torch poses along seam paths using dual surface normals."""

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
        """Sample point indices at specified spacing intervals."""
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
        """Compute torch pose at point index with gap offset and angle rotations."""
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
        """Compute normalized tangent at index using central differences."""
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
        """Compute vector perpendicular to seam, pointing away from secondary piece."""
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
        """Build orthonormal frame (tangent, binormal, normal) from travel and torch directions."""
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
        """Rotate torch around travel axis by work angle toward lean direction."""
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
        """Rotate torch around binormal axis by travel angle."""
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
        """Build pose dict with position, quaternion, and transformation matrix."""
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
