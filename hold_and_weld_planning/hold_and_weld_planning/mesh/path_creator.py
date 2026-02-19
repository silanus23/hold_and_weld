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

"""PathCreator - Smooth and geometrically classify weld paths.

This module handles pure geometric operations on extracted paths:
corner detection on raw paths, spline smoothing per sub-path, and
geometry detection to classify paths as lines, arcs, or complex curves.

TODO: Make torch pose spacing distance-based (point_spacing_mm) instead
of fixed count (num_smooth_points). Compute num_points from seam length
so density is consistent regardless of seam length.
"""

from typing import Dict

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.optimize import least_squares
from sklearn.decomposition import PCA


class PathCreator:
    """Smooth raw vertex paths and detect geometric primitives.

    Takes discrete vertex sequences (from mesh boundary extraction),
    splits at corners on raw geometry, applies spline smoothing per
    sub-path, and recursively detects whether each path represents
    a line, circular arc, or complex curve.
    """

    def process_path(
        self,
        points: np.ndarray,
        is_closed: bool = False,
        num_points: int = 100,
        angle_threshold_deg: float = 45.0,
        corner_window: int = 5,
        line_error_threshold: float = 0.0001,
        circle_error_threshold: float = 0.0001,
    ) -> list[Dict]:
        """Full pipeline: split raw path at corners, smooth each sub-path, detect geometry.

        Flow:
        1. Smooth full path first — stabilizes signal and removes tessellation
           noise, giving corner detection a clean angle distribution to work with
        2. Split smoothed path at corners
        3. Detect geometry type (line/arc/complex)
        4. Flatten complex into arc and line segments

        Args:
            points: (N, 3) raw chained path points
            is_closed: True for closed loops
            num_points: Number of points per smoothed sub-path
            angle_threshold_deg: Corner angle threshold for splitting
            corner_window: Half-window size for local maximum search
            line_error_threshold: Max mean squared error for line fit (m^2)
            circle_error_threshold: Max mean squared error for circle fit (m^2)
        Returns:
            List of dicts with 'type', 'points', and geometry parameters
        """
        smoothed = self._apply_spline_smoothing(
            points, is_closed=is_closed, num_points=num_points
        )

        sub_paths = self._split_path_at_corners(
            smoothed, angle_threshold_deg, corner_window
        )

        result = []
        for sub_path in sub_paths:
            geometry = self._detect_geometry_type(
                sub_path,
                line_error_threshold=line_error_threshold,
                circle_error_threshold=circle_error_threshold,
            )

            if geometry['type'] in ('arc', 'line'):
                result.append(geometry)
            elif geometry['type'] == 'complex':
                arcs, lines = self._extract_segments(geometry)
                result.extend(arcs)
                result.extend(lines)

        return result

    def _extract_segments(self, geometry: Dict) -> tuple[list[Dict], list[Dict]]:
        """Recursively extract arc and line segments from complex geometry.

        Args:
            geometry: Complex geometry dict with nested segments
        Returns:
            Tuple of (arc_segments, line_segments)
        """
        arcs = []
        lines = []

        def recurse(geom):
            if geom['type'] == 'arc':
                arcs.append(geom)
            elif geom['type'] == 'line':
                lines.append(geom)
            elif geom['type'] == 'complex':
                for seg in geom['segments']:
                    recurse(seg)

        recurse(geometry)
        return arcs, lines

    def _split_path_at_corners(
        self,
        points: np.ndarray,
        angle_threshold_deg: float = 45.0,
        corner_window: int = 5,
        min_sub_path_length: int = 5,
    ) -> list[np.ndarray]:
        """Split a path into sub-paths at sharp corner points.

        Uses statistical angle distribution combined with local maximum
        search within a window to find the peak of each turn. This ensures
        each corner is detected once at its sharpest point, not at every
        point in the turn.

                              Larger = more tolerant of spread-out corners.

        Args:
            points: (N, 3) array of path points
            angle_threshold_deg: Minimum angle to consider a corner
            corner_window: Half-window size for local maximum search.
            min_sub_path_length: Minimum points per sub-path
        Returns:
            List of (M, 3) arrays, one per sub-path
        """
        if len(points) < 3:
            return [points]

        # Compute angle change at each interior point
        angles = []
        for i in range(1, len(points) - 1):
            v1 = points[i] - points[i - 1]
            v2 = points[i + 1] - points[i]

            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)

            if norm1 < 1e-10 or norm2 < 1e-10:
                angles.append(0.0)
                continue

            cos_angle = np.clip(np.dot(v1, v2) / (norm1 * norm2), -1.0, 1.0)
            angle_deg = np.degrees(np.arccos(cos_angle))
            angles.append(angle_deg)

        angles = np.array(angles)

        if len(angles) == 0:
            return [points]

        # Statistical threshold: corner must exceed both the hard threshold
        # and be a statistical outlier (mean + 2*std)
        mean_angle = np.mean(angles)
        std_angle = np.std(angles)
        statistical_threshold = mean_angle + 2.0 * std_angle
        effective_threshold = max(angle_threshold_deg, statistical_threshold)

        print(
            f'DEBUG corner: mean={mean_angle:.1f}deg, std={std_angle:.1f}deg, '
            f'effective_threshold={effective_threshold:.1f}deg, window={corner_window}'
        )

        # Find local maxima within window — detects peak of each turn once
        corner_indices = []
        for i, angle in enumerate(angles):
            if angle < effective_threshold:
                continue
            window_start = max(0, i - corner_window)
            window_end = min(len(angles), i + corner_window + 1)
            if angle == np.max(angles[window_start:window_end]):
                corner_indices.append(i + 1)  # offset by 1 for point index

        print(f'DEBUG corner: found {len(corner_indices)} corners at {corner_indices}')

        if not corner_indices:
            return [points]

        # Split at corners — no overlap, corner point belongs to sub-path ending there
        split_indices = [0] + corner_indices + [len(points)]
        sub_paths = []

        for i in range(len(split_indices) - 1):
            start = split_indices[i]
            end = split_indices[i + 1]
            sub_path = points[start:end]

            if len(sub_path) >= min_sub_path_length:
                sub_paths.append(sub_path)
            else:
                if sub_paths:
                    sub_paths[-1] = np.vstack([sub_paths[-1], sub_path[1:]])
                else:
                    sub_paths.append(sub_path)

        print(f'DEBUG corner: split into {len(sub_paths)} sub-paths')

        # The path start is arbitrary (not guaranteed to be at a corner), so the
        # first and last sub-paths may be the same physical side split in two.
        # Merge them if the start point is not actually a corner.
        if len(sub_paths) > 1:
            start_point = points[0]
            tolerance = (
                np.mean(
                    [
                        np.linalg.norm(points[idx] - points[idx - 1])
                        for idx in corner_indices
                    ]
                )
                * 2
            )
            start_is_corner = any(
                np.linalg.norm(start_point - points[idx]) < tolerance
                for idx in corner_indices
            )
            print(
                f'DEBUG corner: start_is_corner={start_is_corner}, tolerance={tolerance:.4f}'
            )
            if not start_is_corner:
                sub_paths[0] = np.vstack([sub_paths[-1], sub_paths[0][1:]])
                sub_paths.pop()
                print(
                    f'DEBUG corner: merged first+last, now {len(sub_paths)} sub-paths'
                )

        return sub_paths

    def _apply_spline_smoothing(
        self,
        points: np.ndarray,
        is_closed: bool = False,
        num_points: int = 100,
        smoothing_factor: float = 0.0,
    ) -> np.ndarray:
        """Apply B-spline smoothing to remove mesh tessellation artifacts.

        Args:
            points: (N, 3) array of 3D points
            is_closed: True for closed loops, False for open paths
            num_points: Number of points in smoothed output
            smoothing_factor: Spline smoothing (0=interpolate exactly)
        Returns:
            Array of smoothed 3D points (num_points, 3)
        """
        vertices = np.asarray(points, dtype=np.float64)

        if len(vertices) < 4:
            return vertices

        x = vertices[:, 0]
        y = vertices[:, 1]
        z = vertices[:, 2]

        try:
            tck, u = splprep([x, y, z], s=smoothing_factor, per=1 if is_closed else 0)
        except Exception as e:
            print(f'Warning: Spline fitting failed: {e}')
            return vertices

        u_new = np.linspace(0, 1, num_points)
        x_smooth, y_smooth, z_smooth = splev(u_new, tck)

        return np.column_stack([x_smooth, y_smooth, z_smooth])

    def _fit_line(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """Fit a 3D line to points using Principal Component Analysis.

        Args:
            points: Array of 3D points (N, 3)
        Returns:
            Tuple of (point_on_line, direction_vector, fit_error)
        """
        if len(points) < 2:
            return points[0], np.array([1, 0, 0]), 0.0

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        pca = PCA(n_components=1)
        pca.fit(centered)
        direction = pca.components_[0]

        projections = np.dot(centered, direction.reshape(-1, 1)) * direction
        residuals = centered - projections
        distances = np.linalg.norm(residuals, axis=1)
        fit_error = np.mean(distances**2)

        return centroid, direction, fit_error

    def _fit_circle(self, points: np.ndarray) -> tuple[np.ndarray, float, float]:
        """Fit a circle to 3D points using least squares optimization.

        Args:
            points: Array of 3D points (N, 3)
        Returns:
            Tuple of (center_3d, radius, fit_error)
        """
        if len(points) < 3:
            return None, None, float('inf')

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        pca = PCA(n_components=2)
        points_2d = pca.fit_transform(centered)

        center_2d_guess = np.mean(points_2d, axis=0)
        radius_guess = np.mean(np.linalg.norm(points_2d - center_2d_guess, axis=1))

        def residuals(params):
            cx, cy, r = params
            distances = np.sqrt(
                (points_2d[:, 0] - cx) ** 2 + (points_2d[:, 1] - cy) ** 2
            )
            return distances - r

        initial_guess = [center_2d_guess[0], center_2d_guess[1], radius_guess]

        try:
            result = least_squares(residuals, initial_guess)
            cx_2d, cy_2d, radius = result.x
            fit_error = np.mean(result.fun**2)

            center_2d = np.array([cx_2d, cy_2d])
            center_3d = pca.inverse_transform(center_2d.reshape(1, -1))[0] + centroid

            return center_3d, abs(radius), fit_error

        except Exception as e:
            print(f'Warning: Circle fitting failed: {e}')
            return None, None, float('inf')

    def _detect_geometry_type(
        self,
        points: np.ndarray,
        line_error_threshold: float = 0.0001,
        circle_error_threshold: float = 0.0001,
        min_segment_points: int = 5,
    ) -> Dict:
        """Recursively detect if points form line, arc, or complex shape.

        Fits line and circle to the full point set. If neither fits well,
        halves the path and recurses on each half independently, building
        a tree of segments until each piece fits a primitive.

        Args:
            points: Array of 3D points (N, 3)
            line_error_threshold: Max error for line fit (m^2)
            circle_error_threshold: Max error for circle fit (m^2)
            min_segment_points: Don't subdivide below this size
        Returns:
            Dict with 'type' ('line'/'arc'/'complex') and parameters
        """
        if len(points) < 3:
            return {'type': 'line', 'points': points}

        line_center, line_direction, line_error = self._fit_line(points)
        circle_center, radius, circle_error = self._fit_circle(points)

        if line_error < line_error_threshold:
            return {
                'type': 'line',
                'points': points,
                'center': line_center,
                'direction': line_direction,
                'error': line_error,
            }

        if circle_error < circle_error_threshold and radius is not None:
            return {
                'type': 'arc',
                'points': points,
                'center': circle_center,
                'radius': radius,
                'error': circle_error,
            }

        if len(points) < min_segment_points * 2:
            return {
                'type': 'line',
                'points': points,
                'center': line_center,
                'direction': line_direction,
                'error': line_error,
                'warning': 'Poor fit, marked as line',
            }

        mid = len(points) // 2

        segment_1 = self._detect_geometry_type(
            points[: mid + 1],
            line_error_threshold,
            circle_error_threshold,
            min_segment_points,
        )

        segment_2 = self._detect_geometry_type(
            points[mid:],
            line_error_threshold,
            circle_error_threshold,
            min_segment_points,
        )

        return {'type': 'complex', 'segments': [segment_1, segment_2]}
