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
"""

from typing import Any, Dict

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


class WeldPlanner:
    """Generate weld poses for seams using dual surface normals.

    Modifies seam objects in-place following the Mutable State pattern.
    Uses normals from both meshes to compute torch orientation without
    needing manual away_from_wall_vector specification.
    """

    def __init__(self, parameters: Dict[str, Any]) -> None:
        """Initialize planner with weld parameters.

            - work_angle_deg: Work angle in degrees (float)
            - travel_angle_deg: Travel angle in degrees (float)
            - gap_mm: Gap distance in millimeters (float)
            - up_vector: World up vector [x, y, z] (default [0, 0, 1])

        Args:
            parameters: Dictionary with keys:
        Raises:
            ValueError: If gap_mm is non-positive
        """
        self.work_angle_rad = np.radians(parameters['work_angle_deg'])
        self.travel_angle_rad = np.radians(parameters['travel_angle_deg'])
        self.gap_m = parameters['gap_mm'] / 1000.0
        self.up_vector = np.array(
            parameters.get('up_vector', [0.0, 0.0, 1.0]), dtype=float
        )
        self.up_vector = self.up_vector / np.linalg.norm(self.up_vector)

        if self.gap_m <= 0:
            raise ValueError(f'gap_mm must be positive, got {parameters["gap_mm"]}')

    def generate_seam(self, seam: Any) -> None:
        """Generate poses for a seam. Modifies seam object in place.

        Args:
            seam: Seam object with smoothed points and normals in config
        Raises:
            RuntimeError: If required data missing from seam.config
            ValueError: If seam geometry is invalid
        """
        # Extract data from seam config
        if 'smoothed_points' not in seam.config:
            raise RuntimeError('Seam missing smoothed_points in config')
        if 'normals_mesh_1' not in seam.config:
            raise RuntimeError('Seam missing normals_mesh_1 in config')
        if 'normals_mesh_2' not in seam.config:
            raise RuntimeError('Seam missing normals_mesh_2 in config')
        if 'is_edge_joint' not in seam.config:
            raise RuntimeError('Seam missing is_edge_joint in config')

        points = seam.config['smoothed_points']
        normals_1 = seam.config['normals_mesh_1']
        normals_2 = seam.config['normals_mesh_2']
        is_edge_joint = seam.config['is_edge_joint']

        if len(points) < 2:
            raise ValueError('Need at least 2 points to generate poses')

        # Generate poses for each point
        poses = []
        for i in range(len(points)):
            tangent = self._compute_tangent(points, i)

            # Get normals at this point
            normal_1 = normals_1[i]
            normal_2 = normals_2[i]

            main_normal, secondary_normal = self._determine_main_normal(
                normal_1, normal_2
            )

            away_from_wall = self._compute_away_vector(
                tangent, main_normal, secondary_normal
            )

            if is_edge_joint:
                # Edge-on-edge: both pieces meet at edges, so torch points into
                # the gap bisector (away from both), and leans along main normal inward.
                main_direction = away_from_wall
                lean_direction = -main_normal
            else:
                # Edge-on-surface: torch points down toward the flat surface (main normal),
                # and leans away from the vertical edge piece.
                main_direction = -main_normal
                lean_direction = away_from_wall

            tangent_base, binormal_base, normal_base = self._build_base_frame(
                tangent, main_direction
            )

            normal_work, binormal_work, tangent_work = self._apply_work_angle(
                normal_base, tangent_base, binormal_base, lean_direction
            )

            normal_final, binormal_final, tangent_final = self._apply_travel_angle(
                normal_work, binormal_work, tangent_work
            )

            # Compute gap offset along bisector direction
            bisector = self._compute_bisector(main_normal, secondary_normal)
            gap_offset = self.gap_m * bisector

            # Final position
            position = points[i] + gap_offset

            # Build pose
            pose = self._build_pose_data(
                position, tangent_final, binormal_final, normal_final, i
            )
            poses.append(pose)

        seam.poses = poses
        seam.is_generated = True

    def _determine_main_normal(
        self,
        normal_1: NDArray,
        normal_2: NDArray,
    ) -> tuple[NDArray, NDArray]:
        """Determine main and secondary normals using world up vector.

        The normal most aligned with the world up vector belongs to the
        main (base/flat) surface. The other is the secondary (edge piece).

        Args:
            normal_1: Normal from mesh_1
            normal_2: Normal from mesh_2
        Returns:
            Tuple of (main_normal, secondary_normal)
        """
        dot_1 = np.dot(normal_1, self.up_vector)
        dot_2 = np.dot(normal_2, self.up_vector)

        if dot_1 >= dot_2:
            return normal_1, normal_2
        else:
            return normal_2, normal_1

    def _compute_bisector(
        self,
        normal_1: NDArray,
        normal_2: NDArray,
    ) -> NDArray:
        """Compute normalized bisector between two normals.

        The bisector points into the corner between the two surfaces,
        which is the correct gap offset direction.

        Args:
            normal_1: First normal vector
            normal_2: Second normal vector
        Returns:
            Normalized bisector vector
        """
        bisector = normal_1 + normal_2
        norm = np.linalg.norm(bisector)

        if norm < 1e-10:
            # Normals are antiparallel - fallback to up vector
            return self.up_vector.copy()

        return bisector / norm

    def _compute_tangent(self, points: NDArray, index: int) -> NDArray:
        """Compute tangent vector at point index.

        Args:
            points: Array of points (N, 3)
            index: Index of point
        Returns:
            Normalized tangent vector
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
        """Compute away_from_wall_vector from dual normals.

        Perpendicular to tangent in main surface plane, pointing away
        from secondary mesh.

        Args:
            tangent: Tangent vector along seam
            main_normal: Normal from main surface
            secondary_normal: Normal from secondary mesh
        Returns:
            Normalized away vector
        """
        # Perpendicular to tangent in main surface plane
        perpendicular = np.cross(main_normal, tangent)
        norm = np.linalg.norm(perpendicular)

        if norm < 1e-10:
            # Tangent parallel to normal - fallback
            perpendicular = np.cross(np.array([0, 0, 1]), tangent)
            norm = np.linalg.norm(perpendicular)
            if norm < 1e-10:
                perpendicular = np.cross(np.array([1, 0, 0]), tangent)
                norm = np.linalg.norm(perpendicular)

        perpendicular = perpendicular / np.linalg.norm(perpendicular)

        # Check direction: should point away from secondary
        if np.dot(perpendicular, secondary_normal) > 0:
            return -perpendicular
        else:
            return perpendicular

    def _build_base_frame(
        self,
        tangent: NDArray,
        main_direction: NDArray,
    ) -> tuple[NDArray, NDArray, NDArray]:
        """Build orthonormal frame from tangent and main direction.

        Args:
            tangent: Direction along seam (travel direction)
            main_direction: Main torch direction
        Returns:
            Tuple of (tangent, binormal, normal)
        """
        normal = main_direction / np.linalg.norm(main_direction)
        binormal = np.cross(normal, tangent)
        binormal = binormal / np.linalg.norm(binormal)

        # Recompute tangent from binormal x normal to enforce strict orthogonality
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

        Args:
            normal: Normal vector (torch direction)
            tangent: Tangent vector (travel direction)
            binormal: Binormal vector
            lean_direction: Target direction for tilting
        Returns:
            Tuple of rotated (normal, binormal, tangent)
        """
        # Determine rotation sign by checking which side of the tangent axis
        # lean_direction falls on relative to normal — avoids testing both directions.
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

        Args:
            normal: Normal vector
            binormal: Binormal vector
            tangent: Tangent vector
        Returns:
            Tuple of rotated (normal, binormal, tangent)
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
        """Build pose data dictionary.

        Args:
            position: 3D position vector
            tangent: Tangent vector (X-axis)
            binormal: Binormal vector (Y-axis)
            normal: Normal vector (Z-axis - torch pointing)
            index: Point index
        Returns:
            Dictionary with pose data
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
