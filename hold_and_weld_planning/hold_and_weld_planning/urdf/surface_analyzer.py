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

This module provides pure geometric operations on Surface objects,
including containment checks, overlap calculations, projections, and
surface relationship analysis.
"""

from typing import List, Optional, Tuple, Dict, Any

import numpy as np
from numpy.typing import NDArray

from ..core.surface import Surface


class SurfaceAnalyzer:
    """Perform geometric queries and calculations on surfaces."""

    @staticmethod
    def project_point_to_surface(
        point: List[float] | NDArray, surface: Surface
    ) -> Tuple[float, float]:
        """Project a point onto a surface plane and get UV coordinates.

        Args:
            point: [x, y, z] point to project.
            surface: Surface object.

        Returns:
            (u_coord, v_coord) tuple in surface local coordinates.
        """
        point = np.array(point)
        center = surface.center
        u_axis = surface.u_axis
        v_axis = surface.v_axis

        vec = point - center
        u_coord = np.dot(vec, u_axis)
        v_coord = np.dot(vec, v_axis)

        return u_coord, v_coord

    @staticmethod
    def check_edge_alignment(
        surface_a: Surface,
        surface_b: Surface,
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
        corners_a = surface_a.corners()
        corners_b = surface_b.corners()

        half_u_b = surface_b.bounds[0] / 2.0
        half_v_b = surface_b.bounds[1] / 2.0

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

        half_u_a = surface_a.bounds[0] / 2.0
        half_v_a = surface_a.bounds[1] / 2.0

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
    def find_touching_pairs(
        main_surfaces: List[Surface],
        secondary_surfaces: List[Surface],
        distance_tol_m: float = 0.015,
        angle_tol_deg: float = 5.0
    ) -> List[Tuple[Surface, Surface]]:
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
                    main_surface.normal, secondary_surface.normal
                )

                if dot_product > -cos_tol:
                    continue

                vec_between = secondary_surface.center - main_surface.center
                distance = abs(np.dot(vec_between, main_surface.normal))

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

    @staticmethod
    def is_point_inside_polygon(
        point: NDArray,
        corners: List[NDArray],
        normal: NDArray,
        tolerance_m: float = 0.001
    ) -> bool:
        """Check if point inside N-sided polygon using ray casting.

        Args:
            point: Point to test [x, y, z]
            corners: List of N corner points defining polygon boundary
            normal: Surface normal (for 2D projection)
            tolerance_m: Tolerance for on-edge points

        Returns:
            True if point is inside or on boundary of polygon
        """
        n = len(corners)
        if n < 3:
            raise ValueError('Polygon must have at least 3 corners')

        # Create 2D coordinate system in polygon plane
        u_axis = corners[1] - corners[0]
        u_axis = u_axis / np.linalg.norm(u_axis)
        v_axis = np.cross(normal, u_axis)
        v_axis = v_axis / np.linalg.norm(v_axis)

        # Project to 2D
        origin = corners[0]
        point_2d = np.array([
            np.dot(point - origin, u_axis),
            np.dot(point - origin, v_axis)
        ])

        corners_2d = []
        for corner in corners:
            corners_2d.append(np.array([
                np.dot(corner - origin, u_axis),
                np.dot(corner - origin, v_axis)
            ]))

        # Ray casting: cast ray to the right, count intersections
        inside = False

        for i in range(n):
            p1 = corners_2d[i]
            p2 = corners_2d[(i + 1) % n]

            # Check if ray crosses this edge
            if ((p1[1] > point_2d[1]) != (p2[1] > point_2d[1])):
                # Calculate X coordinate of intersection
                x_intersect = (
                    p1[0] + (point_2d[1] - p1[1]) / (p2[1] - p1[1]) * (p2[0] - p1[0])
                )
                # If intersection is to the right of point, toggle inside
                if point_2d[0] < x_intersect:
                    inside = not inside

        return inside

    @staticmethod
    def is_point_inside_surface(
        point: NDArray,
        surface: Surface,
        tolerance_m: float = 0.001
    ) -> bool:
        """Check if point inside surface (works for rectangles and polygons).

        Dispatches to appropriate algorithm based on surface data.

        Args:
            point: Point [x, y, z]
            surface: Surface object
            tolerance_m: Tolerance (default 1mm)

        Returns:
            True if point inside surface
        """
        if surface.corners is not None and len(surface.corners) > 0:
            corners = [np.array(c) for c in surface.corners]
            normal = np.array(surface.normal)
            return SurfaceAnalyzer.is_point_inside_polygon(
                point, corners, normal, tolerance_m
            )
        else:
            # Rectangle fallback using UV projection
            u, v = SurfaceAnalyzer.project_point_to_surface(point, surface)
            half_u = surface.bounds[0] / 2.0
            half_v = surface.bounds[1] / 2.0
            return (abs(u) <= half_u + tolerance_m and abs(v) <= half_v + tolerance_m)

    @staticmethod
    def get_surface_corners(
        center: NDArray,
        u_axis: NDArray,
        v_axis: NDArray,
        bounds: Dict[str, float]
    ) -> List[NDArray]:
        """Compute corner points from center, axes, and bounds.

        Args:
            center: Center point [x, y, z] in world coordinates
            u_axis: First basis vector in surface plane (unit vector)
            v_axis: Second basis vector in surface plane (unit vector)
            bounds: Surface bounds with UV coordinate pairs. Keys should be
                'u0', 'v0', 'u1', 'v1', ..., 'uN', 'vN' for N corners.

        Returns:
            List of N corner points in order.

        Raises:
            ValueError: If bounds don't contain matching UV pairs.
        """
        # Extract UV pairs from bounds
        uv_pairs = []
        i = 0
        while f'u{i}' in bounds and f'v{i}' in bounds:
            uv_pairs.append((bounds[f'u{i}'], bounds[f'v{i}']))
            i += 1

        if len(uv_pairs) < 3:
            raise ValueError(
                f'Need at least 3 UV pairs to form corners, got {len(uv_pairs)}'
            )

        # Compute corner points
        corners = []
        for u, v in uv_pairs:
            corner = center + u * u_axis + v * v_axis
            corners.append(corner)

        return corners

    @staticmethod
    def get_overlap_polygon(
        main_surface: Surface,
        secondary_surface: Surface
    ) -> Surface:
        """Compute overlap between two polygonal surfaces using clipping.

        Uses Sutherland-Hodgman polygon clipping algorithm.

        Args:
            main_surface: Main surface with corners
            secondary_surface: Secondary surface with corners

        Returns:
            Overlap Surface object with clipped polygon

        Raises:
            ValueError: If surfaces don't overlap
        """
        print("things come here")
        main_corners = [np.array(c) for c in main_surface.corners]
        secondary_corners = [np.array(c) for c in secondary_surface.corners]

        # Clip secondary polygon against main polygon
        clipped = SurfaceAnalyzer.clip_polygon_sutherland_hodgman(
            secondary_corners,
            main_corners,
            main_surface.normal
        )

        if len(clipped) < 3:
            raise ValueError('Surfaces do not overlap')

        # Compute overlap surface properties
        overlap_center = np.mean(clipped, axis=0)

        # Compute bounds in main's UV space
        u_coords = []
        v_coords = []
        for corner in clipped:
            u, v = SurfaceAnalyzer.project_point_to_surface(corner, main_surface)
            u_coords.append(u)
            v_coords.append(v)

        # TODO: (@silanus23) consider deleting these or see if there is a problem
        overlap_dim_u = max(u_coords) - min(u_coords)
        overlap_dim_v = max(v_coords) - min(v_coords)

        # Compute edges from clipped corners
        edges = Surface.compute_boundary_edges(clipped)

        # Create bounds dict
        u_min = min(u_coords)
        u_max = max(u_coords)
        v_min = min(v_coords)
        v_max = max(v_coords)
        bounds = {
            'u_min': u_min,
            'u_max': u_max,
            'v_min': v_min,
            'v_max': v_max
        }
        print(overlap_center, main_surface.normal, clipped, edges, main_surface.u_axis, main_surface.v_axis, sep="\n")
        print("done")
        return Surface(
            surface_id='overlap',
            center=overlap_center,
            normal=main_surface.normal.copy(),
            corners=clipped,
            edges=edges,
            u_axis=main_surface.u_axis.copy(),
            v_axis=main_surface.v_axis.copy(),
            bounds=bounds
        )

    @staticmethod
    def clip_polygon_sutherland_hodgman(
        subject: List[NDArray],
        clip: List[NDArray],
        normal: NDArray
    ) -> List[NDArray]:
        if len(subject) < 3 or len(clip) < 3:
            return []

        # Instead of calculating axes from clip[0]/clip[1], which can be unstable,
        # use the normal to build a consistent frame.
        def get_axes(n):
            # Find a vector not parallel to n
            v = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
            u = np.cross(v, n)
            u /= np.linalg.norm(u)
            v = np.cross(n, u)
            return u, v

        u_axis, v_axis = get_axes(normal)
        origin = clip[0]

        def to_2d(p):
            return np.array([np.dot(p - origin, u_axis), np.dot(p - origin, v_axis)])

        def to_3d(p2):
            return origin + p2[0] * u_axis + p2[1] * v_axis

        subject_2d = [to_2d(p) for p in subject]
        clip_2d = [to_2d(p) for p in clip]

        # Ensure clipping polygon is Counter-Clockwise (CCW)
        area = 0
        for i in range(len(clip_2d)):
            p1 = clip_2d[i]
            p2 = clip_2d[(i + 1) % len(clip_2d)]
            area += (p2[0] - p1[0]) * (p2[1] + p1[1])
        if area > 0:  # It's Clockwise, reverse it
            clip_2d = clip_2d[::-1]

        output = subject_2d
        epsilon = 1e-9

        for i in range(len(clip_2d)):
            if not output: break

            edge_start = clip_2d[i]
            edge_end = clip_2d[(i + 1) % len(clip_2d)]

            # Inward normal for CCW polygon
            edge_vec = edge_end - edge_start
            edge_normal = np.array([-edge_vec[1], edge_vec[0]])

            input_list = output
            output = []

            for j in range(len(input_list)):
                current = input_list[j]
                previous = input_list[j - 1]

                # Use epsilon to prevent points on the edge from being clipped
                curr_dot = np.dot(current - edge_start, edge_normal)
                prev_dot = np.dot(previous - edge_start, edge_normal)

                current_inside = curr_dot >= -epsilon
                previous_inside = prev_dot >= -epsilon

                if current_inside:
                    if not previous_inside:
                        inter = SurfaceAnalyzer.line_intersection_2d(previous, current, edge_start, edge_end)
                        if inter is not None: output.append(inter)
                    output.append(current)
                elif previous_inside:
                    inter = SurfaceAnalyzer.line_intersection_2d(previous, current, edge_start, edge_end)
                    if inter is not None: output.append(inter)

        return [to_3d(p) for p in output]

    @staticmethod
    def line_intersection_2d(
        p1: NDArray,
        p2: NDArray,
        p3: NDArray,
        p4: NDArray
    ) -> Optional[NDArray]:
        """Find intersection point of two 2D line segments.

        Args:
            p1, p2: First line segment endpoints
            p3, p4: Second line segment endpoints

        Returns:
            Intersection point or None if lines don't intersect
        """
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

        if abs(denom) < 1e-10:
            return None

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom

        return np.array([x1 + t * (x2 - x1), y1 + t * (y2 - y1)])

    @staticmethod
    def compute_polygon_area(
        corners: List[NDArray],
        normal: NDArray
    ) -> float:
        """Compute area of N-sided polygon using shoelace formula.

        Args:
            corners: List of vertices in order (CCW or CW)
            normal: Surface normal vector

        Returns:
            Area in square meters
        """
        n = len(corners)
        if n < 3:
            return 0.0

        # Create 2D coordinate system
        u_axis = corners[1] - corners[0]
        u_axis = u_axis / np.linalg.norm(u_axis)
        v_axis = np.cross(normal, u_axis)
        v_axis = v_axis / np.linalg.norm(v_axis)
        origin = corners[0]

        # Shoelace formula in 2D projection
        area = 0.0

        for i in range(n):
            p1 = corners[i] - origin
            p2 = corners[(i + 1) % n] - origin

            u1 = np.dot(p1, u_axis)
            v1 = np.dot(p1, v_axis)
            u2 = np.dot(p2, u_axis)
            v2 = np.dot(p2, v_axis)

            area += u1 * v2 - u2 * v1

        return abs(area) / 2.0
