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

"""Seam detector - auto-detect weld seam lines from surface geometry.

Geometry-driven seam detection using boundary analysis.
Seams are split at surface boundaries and classified based on edge positions.
"""

import logging
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from .surface_analyzer import SurfaceAnalyzer
from .surface_extractor import SurfaceExtractor
from ..core.surface import Surface
from ..core.line_segment import LineSegment

logger = logging.getLogger(__name__)


class SeamDetector:
    """Auto-detect weld seam lines using pure geometry (boundary-based classification)."""

    def __init__(
        self,
        surface_extractor: SurfaceExtractor,
        surface_analyzer: SurfaceAnalyzer
    ) -> None:
        """Initialize seam detector.

        Args:
            surface_extractor: SurfaceExtractor for getting surfaces from links.
            surface_analyzer: SurfaceAnalyzer for geometric operations.
        """
        self.extractor = surface_extractor
        self.analyzer = surface_analyzer

    def auto_detect_all_seams(
        self,
        main_link: str,
        secondary_link: str,
        secondary_extractor: Optional[SurfaceExtractor] = None,
        min_seam_length_m: float = 0.01
    ) -> List[Dict[str, Any]]:
        """Automatically detect all weld seam lines from geometry.

        Args:
            main_link: Link name in main part.
            secondary_link: Link name in secondary part.
            secondary_extractor: SurfaceExtractor for secondary part (if different URDF).
            min_seam_length_m: Minimum seam length to include (default 10mm).

        Returns:
            List of seam dictionaries, each containing:
                - line_segment: LineSegment object with surfaces
                - is_edge_joint: bool (True if edges meet at boundary)
                - on_surface: 'main', 'secondary', 'both', or 'none'
        """
        if secondary_extractor is None:
            secondary_extractor = self.extractor

        main_surfaces = self.extractor.extract_all_surfaces(main_link)
        secondary_surfaces = secondary_extractor.extract_all_surfaces(
            secondary_link
        )
        touching_pairs = self.analyzer.find_touching_pairs(
            main_surfaces, secondary_surfaces
        )

        if not touching_pairs:
            logger.warning(
                f'No touching surfaces found. '
                f'Main: {len(main_surfaces)} surfaces, '
                f'Secondary: {len(secondary_surfaces)} surfaces'
            )
            return []

        all_seams = []

        for main_surface, secondary_surface in touching_pairs:
            try:
                overlap_surface = self.analyzer.get_overlap_polygon(
                    main_surface, secondary_surface
                )
            except ValueError as e:
                logger.warning(
                    f'Skipping surface pair (main: '
                    f'{main_surface.surface_id}, '
                    f'secondary: '
                    f'{secondary_surface.surface_id}): {e}'
                )
                continue

            # Extract raw seam lines
            seam_lines = self._extract_seam_lines_from_overlap(
                overlap_surface, main_surface, secondary_surface,
                min_seam_length_m
            )

            if not seam_lines:
                continue

            # ALWAYS split seams at boundaries
            for start, end in seam_lines:
                segments = self._split_seam_at_boundaries(
                    start, end, main_surface, secondary_surface
                )

                # Create LineSegment for each valid segment
                for seg in segments:
                    if (seg['length'] < min_seam_length_m or
                            seg['on_surface'] == 'none'):
                        continue

                    # Determine if this is an edge joint
                    seam_on_main_boundary = self._is_seam_on_surface_boundary(
                        seg['start'], seg['end'], main_surface
                    )
                    seam_on_sec_boundary = self._is_seam_on_surface_boundary(
                        seg['start'], seg['end'], secondary_surface
                    )

                    is_edge_joint = seam_on_main_boundary and seam_on_sec_boundary

                    all_seams.append({
                        'line_segment': LineSegment(
                            start=seg['start'],
                            end=seg['end'],
                            main_surface=main_surface,
                            secondary_surface=secondary_surface
                        ),
                        'is_edge_joint': is_edge_joint,
                        'on_surface': seg['on_surface']
                    })

        logger.info(f'Detected {len(all_seams)} seam segments')
        return all_seams

    def _is_seam_on_surface_boundary(
        self,
        seg_start: NDArray,
        seg_end: NDArray,
        surface: Surface,
        tolerance_m: float = 0.005
    ) -> bool:
        """Check if entire seam line lies on a surface boundary edge.

        The seam must lie along a boundary edge, not just have endpoints
        near boundaries.

        Args:
            seg_start: Segment start point [x, y, z].
            seg_end: Segment end point [x, y, z].
            surface: Surface object.
            tolerance_m: Boundary tolerance (default 5mm).

        Returns:
            True if the entire seam line lies on a boundary edge of the
            surface.
        """
        # Project both endpoints to surface UV coordinates
        u_start, v_start = self.analyzer.project_point_to_surface(
            seg_start, surface
        )
        u_end, v_end = self.analyzer.project_point_to_surface(
            seg_end, surface
        )

        width_u = surface.bounds['u_max'] - surface.bounds['u_min']
        half_u = width_u / 2.0

        width_v = surface.bounds['v_max'] - surface.bounds['v_min']
        half_v = width_v / 2.0

        # Check if line lies along U boundary (left or right edge)
        # Both points must be at same U boundary and have consistent U
        # coordinate
        on_left_or_right_edge = (
            (abs(abs(u_start) - half_u) < tolerance_m) and
            (abs(abs(u_end) - half_u) < tolerance_m) and
            (abs(u_start - u_end) < tolerance_m)
        )

        # Check if line lies along V boundary (top or bottom edge)
        # Both points must be at same V boundary and have consistent V
        # coordinate
        on_top_or_bottom_edge = (
            (abs(abs(v_start) - half_v) < tolerance_m) and
            (abs(abs(v_end) - half_v) < tolerance_m) and
            (abs(v_start - v_end) < tolerance_m)
        )

        return on_left_or_right_edge or on_top_or_bottom_edge

    def _split_seam_at_boundaries(
        self,
        seam_start: NDArray,
        seam_end: NDArray,
        main_surface: Surface,
        secondary_surface: Surface
    ) -> List[Dict[str, Any]]:
        """Split seam at surface boundaries.

        Args:
            seam_start: Start point of seam (3D).
            seam_end: End point of seam (3D).
            main_surface: Main surface.
            secondary_surface: Secondary surface.

        Returns:
            List of segment dicts with 'start', 'end', 'on_surface',
            'length'.
        """
        main_corners = main_surface.corners
        secondary_corners = secondary_surface.corners

        main_edges = [
            (main_corners[i], main_corners[(i + 1) % len(main_corners)])
            for i in range(len(main_corners))
        ]
        secondary_edges = [
            (secondary_corners[i], secondary_corners[(i + 1) % len(secondary_corners)])
            for i in range(len(secondary_corners))
        ]

        seam_start_2d = seam_start[:2]
        seam_end_2d = seam_end[:2]

        intersections = []

        for edge_start, edge_end in main_edges + secondary_edges:
            intersection_2d = self._line_segment_intersection_2d(
                seam_start_2d, seam_end_2d,
                edge_start[:2], edge_end[:2]
            )

            if intersection_2d is not None:
                t = (np.linalg.norm(intersection_2d - seam_start_2d) /
                     (np.linalg.norm(seam_end_2d - seam_start_2d) + 1e-10))
                intersection_3d = seam_start + t * (seam_end - seam_start)
                intersections.append(intersection_3d)

        if not intersections:
            on_main = self.analyzer.is_point_inside_surface(
                (seam_start + seam_end) / 2, main_surface, tolerance_m=0.001
            )
            on_secondary = self.analyzer.is_point_inside_surface(
                (seam_start + seam_end) / 2, secondary_surface,
                tolerance_m=0.001
            )

            if on_main and on_secondary:
                on_surface = 'both'
            elif on_main:
                on_surface = 'main'
            elif on_secondary:
                on_surface = 'secondary'
            else:
                on_surface = 'none'

            return [{
                'start': seam_start,
                'end': seam_end,
                'on_surface': on_surface,
                'length': np.linalg.norm(seam_end - seam_start)
            }]

        # Sort and build segments
        intersections.sort(key=lambda p: np.linalg.norm(p - seam_start))
        points = [seam_start] + intersections + [seam_end]

        segments = []
        for i in range(len(points) - 1):
            seg_start = points[i]
            seg_end = points[i + 1]
            seg_midpoint = (seg_start + seg_end) / 2.0

            on_main = self.analyzer.is_point_inside_surface(
                seg_midpoint, main_surface, tolerance_m=0.001
            )
            on_secondary = self.analyzer.is_point_inside_surface(
                seg_midpoint, secondary_surface, tolerance_m=0.001
            )

            if on_main and on_secondary:
                on_surface = 'both'
            elif on_main:
                on_surface = 'main'
            elif on_secondary:
                on_surface = 'secondary'
            else:
                on_surface = 'none'

            segments.append({
                'start': seg_start,
                'end': seg_end,
                'on_surface': on_surface,
                'length': np.linalg.norm(seg_end - seg_start)
            })

        return segments

    @staticmethod
    def _line_segment_intersection_2d(
        p1: NDArray, p2: NDArray, p3: NDArray, p4: NDArray,
        tolerance: float = 1e-10
    ) -> Optional[NDArray]:
        """Find intersection of two 2D line segments."""
        d1 = p2 - p1
        d2 = p4 - p3

        cross = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(cross) < tolerance:
            return None

        t = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / cross
        u = ((p3[0] - p1[0]) * d1[1] - (p3[1] - p1[1]) * d1[0]) / cross

        if 0 <= t <= 1 and 0 <= u <= 1:
            return p1 + t * d1

        return None

    def _extract_seam_lines_from_overlap(
        self,
        overlap_surface: Surface,
        main_surface: Surface,
        secondary_surface: Surface,
        min_seam_length_m: float
    ) -> List[Tuple[NDArray, NDArray]]:
        """Extract seam lines from overlap edges."""
        overlap_corners = overlap_surface.corners

        edges = [
            (overlap_corners[i], overlap_corners[(i + 1) % len(overlap_corners)])
            for i in range(len(overlap_corners))
        ]
        seam_lines = []

        for edge_start, edge_end in edges:
            at_secondary_boundary = self._is_edge_at_surface_boundary(
                edge_start, edge_end, secondary_surface
            )
            at_main_boundary = self._is_edge_at_surface_boundary(
                edge_start, edge_end, main_surface
            )

            # Check if edge overlaps surfaces (handles partial overlap)
            start_in_main = self.analyzer.is_point_inside_surface(
                edge_start, main_surface, tolerance_m=0.001
            )
            end_in_main = self.analyzer.is_point_inside_surface(
                edge_end, main_surface, tolerance_m=0.001
            )
            overlaps_main = start_in_main or end_in_main

            start_in_secondary = self.analyzer.is_point_inside_surface(
                edge_start, secondary_surface, tolerance_m=0.001
            )
            end_in_secondary = self.analyzer.is_point_inside_surface(
                edge_end, secondary_surface, tolerance_m=0.001
            )
            overlaps_secondary = start_in_secondary or end_in_secondary

            is_seam = (
                (at_secondary_boundary and overlaps_main) or
                (at_main_boundary and overlaps_secondary) or
                (at_secondary_boundary and at_main_boundary)
            )

            if is_seam:
                edge_length = np.linalg.norm(edge_end - edge_start)
                if edge_length >= min_seam_length_m:
                    seam_lines.append((edge_start, edge_end))

        return seam_lines

    def _is_edge_at_surface_boundary(
        self,
        edge_start: NDArray,
        edge_end: NDArray,
        surface: Surface,
        tolerance_m: float = 0.005
    ) -> bool:
        """Check if edge lies on surface boundary."""
        width_u = surface.bounds['u_max'] - surface.bounds['u_min']
        half_u = width_u / 2.0

        width_v = surface.bounds['v_max'] - surface.bounds['v_min']
        half_v = width_v / 2.0

        u_start, v_start = self.analyzer.project_point_to_surface(
            edge_start, surface
        )
        u_end, v_end = self.analyzer.project_point_to_surface(
            edge_end, surface
        )

        start_at_boundary = (
            abs(abs(u_start) - half_u) < tolerance_m or
            abs(abs(v_start) - half_v) < tolerance_m
        )
        end_at_boundary = (
            abs(abs(u_end) - half_u) < tolerance_m or
            abs(abs(v_end) - half_v) < tolerance_m
        )

        return start_at_boundary and end_at_boundary
