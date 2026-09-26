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

from dataclasses import dataclass
import logging
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from ..mesh.params import ParamsBase

logger = logging.getLogger(__name__)

# Below this length a vector has no direction worth building a frame from.
DEGENERATE_LENGTH = 1e-10


@dataclass
class WeldPlannerParams(ParamsBase):
    """Weld parameters for WeldPlanner; all but the spacing are required."""

    work_angle_deg: float
    travel_angle_deg: float
    gap_mm: float
    waypoint_spacing_mm: float = 10.0

    REQUIRED = ('work_angle_deg', 'travel_angle_deg', 'gap_mm')

    @classmethod
    def from_dict(cls, params: dict[str, Any] | None):
        """Build from a config dict, naming any required key that is missing.

        Raises:
            ValueError: If params is not a dict, a required key is missing, a value cannot be
                coerced, or a value fails a constraint.
        """
        if isinstance(params, dict):
            missing = [key for key in cls.REQUIRED if key not in params]
            if missing:
                raise ValueError(f'Missing required weld parameter(s): {missing}')
        elif params is not None:
            raise ValueError(f'params must be a dict, got {type(params).__name__}')
        else:
            raise ValueError(f'Missing required weld parameter(s): {list(cls.REQUIRED)}')
        return super().from_dict(params)

    def __post_init__(self) -> None:
        """Reject values that would place the torch nowhere sensible.

        Raises:
            ValueError: If a parameter is out of range.
        """
        for key in ('gap_mm', 'waypoint_spacing_mm'):
            if not getattr(self, key) > 0.0:
                raise ValueError(f'{key} must be > 0, got {getattr(self, key)}')

        # At 90 degrees either tilt lays the torch flat along a surface; past it, into the part.
        for key in ('work_angle_deg', 'travel_angle_deg'):
            if not abs(getattr(self, key)) < 90.0:
                raise ValueError(
                    f'{key} must be strictly between -90 and 90, got {getattr(self, key)}')


class WeldPlanner:
    """Generate weld torch poses along seam paths using dual surface normals."""

    def __init__(self, parameters: dict[str, Any]) -> None:
        """Initialize planner with weld parameters.

        Args:
            parameters: Dictionary with keys:
                - work_angle_deg: Work angle in degrees (torch tilt perpendicular to travel)
                - travel_angle_deg: Travel angle in degrees (torch tilt along travel)
                - gap_mm: Gap distance from seam in millimeters
                - waypoint_spacing_mm: Distance between waypoints (default 10mm)
                Other keys are ignored, so the job's shared parameter dict can
                be handed over whole.

        Raises:
            ValueError: If a required key is missing, a value is not a finite
                number, gap_mm or waypoint_spacing_mm is non-positive, or an
                angle is not strictly between -90 and 90 degrees.
        """
        cfg = WeldPlannerParams.from_dict(parameters)
        self.work_angle_rad = np.radians(cfg.work_angle_deg)
        self.travel_angle_rad = np.radians(cfg.travel_angle_deg)
        self.gap_m = cfg.gap_mm / 1000.0
        self.waypoint_spacing_m = cfg.waypoint_spacing_mm / 1000.0

        logger.debug(
            f'WeldPlanner initialized: work_angle={cfg.work_angle_deg}°, '
            f'travel_angle={cfg.travel_angle_deg}°, '
            f'gap={cfg.gap_mm}mm, '
            f'waypoint_spacing={cfg.waypoint_spacing_mm}mm'
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
            ValueError: If arrays have invalid lengths, no point has a usable
                normal, or every point coincides so there is no tangent

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

        logger.debug(f'Generating poses for seam with {len(points)} points')

        self._validate_arrays(points, normals_main, normals_secondary)
        normals_main = self._fill_missing_normals(normals_main, 'normals_main')
        normals_secondary = self._fill_missing_normals(normals_secondary, 'normals_secondary')

        sampled_indices = self._sample_by_distance(points, self.waypoint_spacing_m)
        logger.debug(f'Sampled {len(sampled_indices)} waypoints from {len(points)} points')

        poses = []
        for idx in sampled_indices:
            pose = self._compute_pose_at_index(
                points, normals_main, normals_secondary, is_edge_joint, idx
            )
            poses.append(pose)

        seam.poses = poses
        seam.is_generated = True
        logger.info(
            f'Generated {len(poses)} poses for {seam.segment_type} seam'
            f' ({seam.length()*1000:.1f}mm)'
        )

    def _validate_arrays(
        self, points: NDArray, normals_main: NDArray, normals_secondary: NDArray
    ) -> None:
        """Validate input arrays have consistent lengths and sufficient points."""
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

    @staticmethod
    def _fill_missing_normals(normals: NDArray, name: str) -> NDArray:
        """Give each point without a usable normal that of the nearest point with one.

        An extractor reports a normal it could not find as zeros. Left in, one such point makes
        its pose NaN and costs the whole seam; along a seam the surface turns slowly, so the
        neighbour's normal is the better answer.

        Raises:
            ValueError: If no point has a usable normal.
        """
        normals = np.asarray(normals, dtype=float)
        usable = np.isfinite(normals).all(axis=1) & (
            np.linalg.norm(np.nan_to_num(normals), axis=1) > DEGENERATE_LENGTH)
        if usable.all():
            return normals
        if not usable.any():
            raise ValueError(f'Seam has no usable normal in {name}')

        good = np.nonzero(usable)[0]
        bad = np.nonzero(~usable)[0]
        nearest = good[np.argmin(np.abs(bad[:, None] - good[None, :]), axis=1)]
        filled = normals.copy()
        filled[bad] = normals[nearest]
        logger.warning(
            f'{len(bad)} of {len(normals)} point(s) have no usable {name}; '
            'each takes the normal of the nearest point that has one'
        )
        return filled

    def _sample_by_distance(self, points: NDArray, spacing: float) -> list[int]:
        """Return point indices sampled at specified spacing along path."""
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

        if len(sampled) == len(points):
            logger.debug(
                f'waypoint_spacing ({self.waypoint_spacing_m*1000:.1f}mm) is smaller than '
                f'typical point spacing — all {len(points)} points sampled'
            )

        return sampled

    def _compute_pose_at_index(
        self,
        points: NDArray,
        normals_main: NDArray,
        normals_secondary: NDArray,
        is_edge_joint: bool,
        index: int,
    ) -> dict[str, Any]:
        """Compute torch pose at index with gap offset and work/travel angle rotations."""
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
            logger.debug(f'Degenerate gap offset direction at index {index}, using main_normal')
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
        """Compute normalized tangent vector at index using central differences.

        Each side takes the nearest point that does not coincide with this one, so a repeated
        sample still gets the seam's direction.

        Raises:
            ValueError: If every point coincides with this one.
        """
        here = points[index]
        distance = np.linalg.norm(points - here, axis=1)
        apart = distance > DEGENERATE_LENGTH

        ahead = np.nonzero(apart[index + 1:])[0]
        behind = np.nonzero(apart[:index])[0]
        forward = points[index + 1 + ahead[0]] if len(ahead) else here
        backward = points[behind[-1]] if len(behind) else here

        tangent = forward - backward
        norm = np.linalg.norm(tangent)
        if norm < DEGENERATE_LENGTH:
            raise ValueError(
                f'Cannot compute tangent at index {index}: every seam point coincides with it')

        return tangent / norm

    def _compute_away_vector(
        self, tangent: NDArray, main_normal: NDArray, secondary_normal: NDArray
    ) -> NDArray:
        """Compute vector perpendicular to seam pointing away from secondary piece."""
        perpendicular = np.cross(main_normal, tangent)
        norm = np.linalg.norm(perpendicular)

        if norm < 1e-10:
            logger.debug('Tangent nearly parallel to main_normal, using fallback axes')
            perpendicular = np.cross(np.array([0, 0, 1]), tangent)
            norm = np.linalg.norm(perpendicular)
            if norm < 1e-10:
                perpendicular = np.cross(np.array([1, 0, 0]), tangent)
                norm = np.linalg.norm(perpendicular)
                if norm < 1e-10:
                    # All cross-product fallbacks failed — tangent is degenerate. Compute a
                    # guaranteed-perpendicular vector via cross with [0,1,0], then cross that with
                    # tangent to ensure perpendicularity.
                    logger.warning(
                        'Degenerate tangent in _compute_away_vector,'
                        ' using [0, 1, 0] cross tangent'
                    )
                    fallback = np.cross(np.array([0.0, 1.0, 0.0]), tangent)
                    fallback_norm = np.linalg.norm(fallback)
                    perpendicular = (
                        fallback / fallback_norm
                        if fallback_norm > 1e-10
                        else np.array([0.0, 1.0, 0.0])
                    )

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
        """Build orthonormal frame (tangent, binormal, normal) from tangent and torch direction."""
        normal = main_direction / np.linalg.norm(main_direction)
        binormal = np.cross(normal, tangent)
        binormal_norm = np.linalg.norm(binormal)
        if binormal_norm < 1e-10:
            raise ValueError(
                'Cannot build base frame: normal and tangent are parallel '
                '(away_from_wall vector is collinear with seam direction)'
            )
        binormal = binormal / binormal_norm

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
        """Rotate torch around tangent axis by work angle toward lean direction."""
        sign = np.sign(np.dot(np.cross(normal, lean_direction), tangent))
        if sign == 0:
            # normal and lean_direction are coplanar with tangent — default to positive rotation
            sign = 1.0
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
        """Rotate torch around binormal axis by travel angle (pushes/pulls along weld direction).

        Pushes/pulls the torch along the weld direction.
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
    ) -> dict[str, Any]:
        """Build pose dictionary with position, quaternion, and 4x4 transform matrix."""
        rot_matrix = np.column_stack([tangent, binormal, normal])

        if not np.all(np.isfinite(rot_matrix)):
            raise ValueError(
                f'Non-finite values in rotation matrix at index {index}: '
                f'likely NaN/Inf propagated from degenerate normal or tangent'
            )

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
