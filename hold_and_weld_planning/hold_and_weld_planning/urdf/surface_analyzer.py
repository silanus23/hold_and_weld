# Copyright 2026 Berkan Tali
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

"""Surface analyzer - geometric queries and calculations on surfaces.

This module provides pure geometric operations on surface dictionaries,
including containment checks, overlap calculations, projections, and
surface relationship analysis.
"""

from typing import Any, Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray


class SurfaceAnalyzer:
    """Perform geometric queries and calculations on surfaces."""

    @staticmethod
    def get_surface_corners(surface: Dict[str, Any]) -> List[NDArray]:
        """Get the 4 corner points of a rectangular surface.

        Args:
            surface: Surface dictionary with center, u_axis, v_axis, bounds.

        Returns:
            4 corner points as np.arrays in clockwise order.
        """
        center = surface['center']
        u_axis = surface['u_axis']
        v_axis = surface['v_axis']
        half_u = surface['bounds'][0] / 2.0
        half_v = surface['bounds'][1] / 2.0

        corners = [
            center + half_u * u_axis + half_v * v_axis,
            center + half_u * u_axis - half_v * v_axis,
            center - half_u * u_axis - half_v * v_axis,
            center - half_u * u_axis + half_v * v_axis,
        ]

        return corners

    @staticmethod
    def project_point_to_surface(
        point: List[float] | NDArray, surface: Dict[str, Any]
    ) -> Tuple[float, float]:
        """Project a point onto a surface plane and get UV coordinates.

        Args:
            point: [x, y, z] point to project.
            surface: Surface dictionary.

        Returns:
            (u_coord, v_coord) tuple in surface local coordinates.
        """
        point = np.array(point)
        center = surface['center']
        u_axis = surface['u_axis']
        v_axis = surface['v_axis']

        vec = point - center
        u_coord = np.dot(vec, u_axis)
        v_coord = np.dot(vec, v_axis)

        return u_coord, v_coord

    @staticmethod
    def is_point_inside_surface(
        point: NDArray,
        surface: Dict[str, Any],
        tolerance_m: float = 0.001
    ) -> bool:
        """Check if a point lies inside a surface's bounds.

        Args:
            point: Point to check [x, y, z].
            surface: Surface dictionary.
            tolerance_m: Tolerance for bounds check (default 1mm).

        Returns:
            True if point is inside surface bounds.
        """
        u, v = SurfaceAnalyzer.project_point_to_surface(point, surface)
        half_u = surface['bounds'][0] / 2.0
        half_v = surface['bounds'][1] / 2.0

        return (abs(u) <= half_u + tolerance_m and abs(v) <= half_v + tolerance_m)

    @staticmethod
    def check_containment(
        inner_surface: Dict[str, Any],
        outer_surface: Dict[str, Any],
        tolerance_m: float = 0.001
    ) -> bool:
        """Check if inner_surface is completely contained within outer_surface.

        Args:
            inner_surface: Surface that might be inside.
            outer_surface: Surface that might contain the other.
            tolerance_m: Edge tolerance (default 1mm).

        Returns:
            True if inner_surface fits completely inside outer_surface.
        """
        inner_corners = SurfaceAnalyzer.get_surface_corners(inner_surface)
        outer_half_u = outer_surface['bounds'][0] / 2.0
        outer_half_v = outer_surface['bounds'][1] / 2.0

        for corner in inner_corners:
            u, v = SurfaceAnalyzer.project_point_to_surface(corner, outer_surface)
            if abs(u) > outer_half_u + tolerance_m:
                return False
            if abs(v) > outer_half_v + tolerance_m:
                return False

        return True

    @staticmethod
    def check_edge_alignment(
        surface_a: Dict[str, Any],
        surface_b: Dict[str, Any],
        tolerance_m: float = 0.01
    ) -> str:
        """Check if surface edges align or one extends past the other.

        Args:
            surface_a: First surface.
            surface_b: Second surface.
            tolerance_m: Edge alignment tolerance (default 10mm).

        Returns:
            'aligned' if edges meet, 'partial' otherwise.
        """
        corners_a = SurfaceAnalyzer.get_surface_corners(surface_a)
        corners_b = SurfaceAnalyzer.get_surface_corners(surface_b)

        half_u_b = surface_b['bounds'][0] / 2.0
        half_v_b = surface_b['bounds'][1] / 2.0

        coords_a = []
        for corner in corners_a:
            u, v = SurfaceAnalyzer.project_point_to_surface(corner, surface_b)
            coords_a.append((u, v))

        reaches_u_edge = any(
            abs(u) >= half_u_b - tolerance_m for u, v in coords_a
        )
        reaches_v_edge = any(
            abs(v) >= half_v_b - tolerance_m for u, v in coords_a
        )

        half_u_a = surface_a['bounds'][0] / 2.0
        half_v_a = surface_a['bounds'][1] / 2.0

        coords_b = []
        for corner in corners_b:
            u, v = SurfaceAnalyzer.project_point_to_surface(corner, surface_a)
            coords_b.append((u, v))

        b_reaches_u_edge = any(
            abs(u) >= half_u_a - tolerance_m for u, v in coords_b
        )
        b_reaches_v_edge = any(
            abs(v) >= half_v_a - tolerance_m for u, v in coords_b
        )

        if reaches_u_edge or reaches_v_edge or b_reaches_u_edge or b_reaches_v_edge:
            return 'aligned'
        return 'partial'

    @staticmethod
    def get_overlap_surface(
        main_surface: Dict[str, Any],
        secondary_surface: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Compute the overlapping region between two touching surfaces.

        Args:
            main_surface: Surface dict from main part.
            secondary_surface: Surface dict from secondary part.

        Returns:
            Overlap surface dictionary.

        Raises:
            ValueError: If surfaces don't overlap.
        """
        secondary_corners = SurfaceAnalyzer.get_surface_corners(secondary_surface)

        u_coords = []
        v_coords = []
        for corner in secondary_corners:
            u, v = SurfaceAnalyzer.project_point_to_surface(corner, main_surface)
            u_coords.append(u)
            v_coords.append(v)

        sec_u_min, sec_u_max = min(u_coords), max(u_coords)
        sec_v_min, sec_v_max = min(v_coords), max(v_coords)

        main_half_u = main_surface['bounds'][0] / 2.0
        main_half_v = main_surface['bounds'][1] / 2.0

        overlap_u_min = max(sec_u_min, -main_half_u)
        overlap_u_max = min(sec_u_max, main_half_u)
        overlap_v_min = max(sec_v_min, -main_half_v)
        overlap_v_max = min(sec_v_max, main_half_v)

        if overlap_u_min >= overlap_u_max or overlap_v_min >= overlap_v_max:
            raise ValueError('Surfaces do not overlap')

        overlap_dim_u = overlap_u_max - overlap_u_min
        overlap_dim_v = overlap_v_max - overlap_v_min
        overlap_center_u = (overlap_u_min + overlap_u_max) / 2.0
        overlap_center_v = (overlap_v_min + overlap_v_max) / 2.0

        overlap_center = (
            main_surface['center']
            + overlap_center_u * main_surface['u_axis']
            + overlap_center_v * main_surface['v_axis']
        )

        return {
            'center': overlap_center,
            'normal': main_surface['normal'].copy(),
            'u_axis': main_surface['u_axis'].copy(),
            'v_axis': main_surface['v_axis'].copy(),
            'bounds': [overlap_dim_u, overlap_dim_v],
            'area': overlap_dim_u * overlap_dim_v
        }

    @staticmethod
    def find_touching_pairs(
        main_surfaces: List[Dict[str, Any]],
        secondary_surfaces: List[Dict[str, Any]],
        distance_tol_m: float = 0.015,
        angle_tol_deg: float = 5.0
    ) -> List[Tuple[Dict[str, Any], Dict[str, Any]]]:
        """Find pairs of surfaces that are touching between two parts.

        Args:
            main_surfaces: List of surfaces from main part.
            secondary_surfaces: List of surfaces from secondary part.
            distance_tol_m: Maximum distance between planes (default 15mm).
            angle_tol_deg: Maximum angle difference for parallel normals (default 5°).

        Returns:
            List of touching pairs, each is (main_surface, secondary_surface) tuple.
        """
        touching_pairs = []
        cos_tol = np.cos(np.radians(angle_tol_deg))

        for main_surface in main_surfaces:
            for secondary_surface in secondary_surfaces:
                # Check if normals are anti-parallel (facing each other)
                dot_product = np.dot(
                    main_surface['normal'], secondary_surface['normal']
                )

                if dot_product > -cos_tol:
                    # Normals not facing each other
                    continue

                # Check distance between planes
                vec_between = secondary_surface['center'] - main_surface['center']
                distance = abs(np.dot(vec_between, main_surface['normal']))

                if distance <= distance_tol_m:
                    touching_pairs.append((main_surface, secondary_surface))

        return touching_pairs

    @staticmethod
    def find_closest_pair_to_seam(
        touching_pairs: List[Tuple[Dict[str, Any], Dict[str, Any]]],
        seam_line: Any,
        tolerance_m: float = 0.05
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """Find which touching surface pair is closest to the weld seam.

        Args:
            touching_pairs: List of (main_surface, secondary_surface) tuples.
            seam_line: LineSegment object representing the weld seam.
            tolerance_m: Maximum distance from seam to surface (default 50mm).

        Returns:
            (main_surface, secondary_surface) tuple closest to seam.

        Raises:
            ValueError: If no touching pair is within tolerance of seam.
        """
        if not touching_pairs:
            raise ValueError('No touching surface pairs found')

        min_distance = float('inf')
        closest_pair = None

        for main_surface, secondary_surface in touching_pairs:
            dist_start = SurfaceAnalyzer.point_to_plane_distance(
                seam_line.start, main_surface['center'], main_surface['normal']
            )
            dist_end = SurfaceAnalyzer.point_to_plane_distance(
                seam_line.end, main_surface['center'], main_surface['normal']
            )

            avg_distance = (dist_start + dist_end) / 2.0

            if avg_distance < min_distance:
                min_distance = avg_distance
                closest_pair = (main_surface, secondary_surface)

        if min_distance > tolerance_m:
            raise ValueError(
                f'No touching surface within {tolerance_m*1000:.1f}mm of seam '
                f'(closest: {min_distance*1000:.1f}mm)'
            )

        return closest_pair

    @staticmethod
    def point_to_plane_distance(
        point: List[float] | NDArray,
        plane_center: NDArray,
        plane_normal: NDArray
    ) -> float:
        """Calculate perpendicular distance from point to plane.

        Args:
            point: Point [x, y, z].
            plane_center: Center of plane.
            plane_normal: Normal vector of plane.

        Returns:
            Absolute distance from point to plane.
        """
        vec = np.array(point) - np.array(plane_center)
        return abs(np.dot(vec, plane_normal))
