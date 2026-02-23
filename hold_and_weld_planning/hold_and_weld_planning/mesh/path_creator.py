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

"""

from typing import Dict, List

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.optimize import least_squares
from sklearn.decomposition import PCA


class PathCreator:
    """Smooth raw vertex paths and detect geometric primitives."""

    def process_path(
        self,
        points: np.ndarray,
        is_closed: bool = False,
        num_points: int = 100,
        line_error_threshold: float = 0.0001,
        circle_error_threshold: float = 0.0001,
        spline_smoothing_factor: float = 0.0,
    ) -> List[Dict]:
        """Process path: detect corners, smooth, classify, merge primitives.

        Args:
            points: (N, 3) raw path points
            is_closed: True for closed loops
            num_points: Points per smoothed segment
            line_error_threshold: Unused, kept for compatibility
            circle_error_threshold: Unused, kept for compatibility
            spline_smoothing_factor: Spline smoothing factor

        Returns:
            List of geometry dicts with 'type', 'points', and parameters
        """
        points_cleaned = self._remove_outliers(points, std_threshold=5.0)

        sub_paths = self._split_path_at_corners(points_cleaned, min_sub_path_length=10)

        segments_with_raw = []

        for raw_segment in sub_paths:
            smoothed = self._apply_spline_smoothing(
                raw_segment,
                is_closed=False,
                num_points=num_points,
                smoothing_factor=spline_smoothing_factor,
            )

            geometry_type, params = self._classify_segment(smoothed)

            if geometry_type == 'arc':
                geometry = {
                    'type': 'arc',
                    'points': smoothed,
                    'center': params['center'],
                    'radius': params['radius'],
                    'error': params['error'],
                }
            else:
                geometry = {
                    'type': 'line',
                    'points': smoothed,
                    'center': params['center'],
                    'direction': params['direction'],
                    'error': params['error'],
                }

            segments_with_raw.append((geometry, raw_segment))

        # Merge first+last if closed path split arbitrarily
        if len(segments_with_raw) > 1:
            first_geom = segments_with_raw[0][0]
            last_geom = segments_with_raw[-1][0]

            if self._should_merge_first_last(sub_paths, first_geom, last_geom):
                first_raw = segments_with_raw[0][1]
                last_raw = segments_with_raw[-1][1]

                combined_raw = np.vstack([last_raw, first_raw[1:]])
                smoothed_combined = self._apply_spline_smoothing(
                    combined_raw, is_closed=False, num_points=num_points
                )

                geom_type, params = self._classify_segment(smoothed_combined)

                if geom_type == 'arc':
                    merged_geom = {
                        'type': 'arc',
                        'points': smoothed_combined,
                        'center': params['center'],
                        'radius': params['radius'],
                        'error': params['error'],
                    }
                else:
                    merged_geom = {
                        'type': 'line',
                        'points': smoothed_combined,
                        'center': params['center'],
                        'direction': params['direction'],
                        'error': params['error'],
                    }

                segments_with_raw = [(merged_geom, combined_raw)] + segments_with_raw[1:-1]

        result = self._merge_consecutive_arcs(segments_with_raw, radius_tolerance=0.05)
        result = self._merge_consecutive_lines(result, angle_tolerance_deg=10.0, max_merge_error=0.001)
        result = self._merge_small_lines_into_arcs(result, min_line_size=70, max_merge_error=0.01)

        return result

    def _should_merge_first_last(
        self,
        sub_paths: List[np.ndarray],
        first_geom: Dict,
        last_geom: Dict,
    ) -> bool:
        """Check if first and last segments should merge (closed path split arbitrarily).

        Args:
            sub_paths: List of sub-path point arrays
            first_geom: First segment geometry dict
            last_geom: Last segment geometry dict

        Returns:
            True if segments should be merged, False otherwise
        """

        if len(sub_paths) <= 1:
            return False

        if first_geom['type'] != last_geom['type']:
            return False

        if first_geom['type'] == 'line':
            dir1 = first_geom['direction']
            dir2 = last_geom['direction']
            cos_angle = abs(np.dot(dir1, dir2))
            angle_deg = np.degrees(np.arccos(np.clip(cos_angle, 0, 1)))

            if angle_deg > 10:
                return False

        elif first_geom['type'] == 'arc':
            r1 = first_geom['radius']
            r2 = last_geom['radius']

            if abs(r1 - r2) > 0.05:
                return False

        first_path = sub_paths[0]
        last_path = sub_paths[-1]

        gap = np.linalg.norm(last_path[-1] - first_path[0])

        if gap > 0.01:
            return False

        return True

    def _split_path_at_corners(
        self,
        points: np.ndarray,
        min_sub_path_length: int = 10,
    ) -> List[np.ndarray]:
        """Detect corners and split path.

        Args:
            points: Input path points (N, 3)
            min_sub_path_length: Minimum points per sub-path

        Returns:
            List of sub-path point arrays
        """

        if len(points) < 20:
            return [points]

        angle_corners = self._detect_corners_by_angle(points, min_angle=25, window=10)
        curvature_corners = self._detect_corners_by_curvature(points, window=10, threshold=0.8)

        corner_indices = self._combine_corner_detections(
            [angle_corners, curvature_corners],
            min_agreement=1,
            tolerance=5,
        )

        if not corner_indices:
            return [points]

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

        return sub_paths

    def _detect_corners_by_angle(
        self,
        points: np.ndarray,
        min_angle: float = 25,
        window: int = 10,
    ) -> List[int]:
        """Detect corners by angle change.

        Args:
            points: Input path points (N, 3)
            min_angle: Minimum angle change in degrees to detect corner
            window: Number of points to look ahead/behind

        Returns:
            List of corner point indices
        """

        corners = []

        for i in range(window, len(points) - window):
            v_before = points[i] - points[i - window]
            v_after = points[i + window] - points[i]

            norm_before = np.linalg.norm(v_before)
            norm_after = np.linalg.norm(v_after)

            if norm_before < 1e-10 or norm_after < 1e-10:
                continue

            cos_angle = np.dot(v_before, v_after) / (norm_before * norm_after)
            angle_deg = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))

            if angle_deg > min_angle:
                corners.append(i)

        return self._filter_to_local_maxima(corners, points, window=5)

    def _detect_corners_by_curvature(
        self,
        points: np.ndarray,
        window: int = 10,
        threshold: float = 0.8,
    ) -> List[int]:
        """Detect corners by curvature.

        Args:
            points: Input path points (N, 3)
            window: Number of points for curvature calculation
            threshold: Normalized curvature threshold (0-1)

        Returns:
            List of corner point indices
        """

        if len(points) < window * 2:
            return []

        curvatures = []

        for i in range(len(points)):
            if i < window or i >= len(points) - window:
                curvatures.append(0.0)
                continue

            v1 = points[i] - points[i - window]
            v2 = points[i + window] - points[i]

            curvature = np.linalg.norm(v2 - v1) / (2 * window)
            curvatures.append(curvature)

        curvatures = np.array(curvatures)

        if len(curvatures) == 0 or np.max(curvatures) < 1e-10:
            return []

        max_curvature = np.max(curvatures)
        normalized = curvatures / max_curvature

        corners = []
        for i in range(window, len(normalized) - window):
            if normalized[i] > threshold:
                local_start = max(0, i - 5)
                local_end = min(len(normalized), i + 6)
                if normalized[i] == np.max(normalized[local_start:local_end]):
                    corners.append(i)

        return corners

    def _combine_corner_detections(
        self,
        corner_lists: List[List[int]],
        min_agreement: int = 1,
        tolerance: int = 5,
    ) -> List[int]:
        """Combine corner detections from multiple methods.

        Args:
            corner_lists: List of corner detection results from different methods
            min_agreement: Minimum number of methods that must agree
            tolerance: Maximum distance between corners to be considered the same

        Returns:
            List of combined corner indices
        """

        if not corner_lists:
            return []

        all_corners = []
        for corners in corner_lists:
            all_corners.extend(corners)

        if not all_corners:
            return []

        all_corners = sorted(all_corners)

        groups = []
        current_group = [all_corners[0]]

        for corner in all_corners[1:]:
            if corner - current_group[-1] <= tolerance:
                current_group.append(corner)
            else:
                groups.append(current_group)
                current_group = [corner]
        groups.append(current_group)

        verified = []
        for group in groups:
            if len(group) >= min_agreement:
                verified.append(int(np.median(group)))

        return sorted(verified)

    def _filter_to_local_maxima(
        self,
        indices: List[int],
        points: np.ndarray,
        window: int = 5,
    ) -> List[int]:
        """Filter to local maxima of angle change.

        Args:
            indices: Candidate corner indices
            points: Input path points (N, 3)
            window: Window size for finding local maxima

        Returns:
            List of filtered corner indices that are local maxima
        """

        if not indices:
            return []

        angles = {}
        for idx in indices:
            if idx < 10 or idx >= len(points) - 10:
                continue

            v1 = points[idx] - points[idx - 10]
            v2 = points[idx + 10] - points[idx]

            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)

            if norm1 < 1e-10 or norm2 < 1e-10:
                continue

            cos_angle = np.dot(v1, v2) / (norm1 * norm2)
            angle = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))
            angles[idx] = angle

        filtered = []
        for idx in sorted(angles.keys()):
            is_maximum = True
            for other_idx in angles.keys():
                if abs(other_idx - idx) <= window and other_idx != idx:
                    if angles[other_idx] > angles[idx]:
                        is_maximum = False
                        break

            if is_maximum:
                filtered.append(idx)

        return filtered

    def _merge_consecutive_arcs(
        self,
        segments_with_raw: List[tuple],
        radius_tolerance: float = 0.05,
    ) -> List[Dict]:
        """Merge consecutive arcs with similar radii.

        Args:
            segments_with_raw: List of (geometry_dict, raw_points) tuples
            radius_tolerance: Maximum radius difference to merge arcs

        Returns:
            List of merged geometry dictionaries
        """

        if len(segments_with_raw) <= 1:
            return [seg[0] for seg in segments_with_raw]

        merged = []
        i = 0

        while i < len(segments_with_raw):
            current_geom, current_raw = segments_with_raw[i]

            if current_geom['type'] != 'arc':
                merged.append(current_geom)
                i += 1
                continue

            arc_group_geoms = [current_geom]
            arc_group_raw = [current_raw]
            j = i + 1

            while j < len(segments_with_raw):
                next_geom, next_raw = segments_with_raw[j]

                if next_geom['type'] != 'arc':
                    break

                radius_diff = abs(current_geom['radius'] - next_geom['radius'])

                if radius_diff < radius_tolerance:
                    arc_group_geoms.append(next_geom)
                    arc_group_raw.append(next_raw)
                    j += 1
                else:
                    break

            if len(arc_group_geoms) > 1:
                combined_raw = np.vstack(arc_group_raw)
                center, radius, error = self._fit_circle(combined_raw)

                smoothed = self._apply_spline_smoothing(
                    combined_raw, is_closed=False, num_points=100
                )

                merged_geom = {
                    'type': 'arc',
                    'points': smoothed,
                    'center': center,
                    'radius': radius,
                    'error': error,
                }

                merged.append(merged_geom)
                i = j
            else:
                merged.append(current_geom)
                i += 1

        return merged

    def _merge_consecutive_lines(
        self,
        segments: List[Dict],
        angle_tolerance_deg: float = 10.0,
        max_merge_error: float = 0.001,
    ) -> List[Dict]:
        """Merge consecutive lines if collinear.

        Args:
            segments: List of geometry dictionaries
            angle_tolerance_deg: Maximum angle difference in degrees to merge
            max_merge_error: Maximum fit error to allow merge

        Returns:
            List of merged geometry dictionaries
        """

        iteration = 0
        max_iterations = 20

        while iteration < max_iterations:
            iteration += 1
            merge_found = False

            for i in range(len(segments) - 1):
                seg1 = segments[i]
                seg2 = segments[i + 1]

                if seg1['type'] != 'line' or seg2['type'] != 'line':
                    continue

                dir1 = seg1['direction']
                dir2 = seg2['direction']

                cos_angle = abs(np.dot(dir1, dir2))
                angle_deg = np.degrees(np.arccos(np.clip(cos_angle, 0, 1)))

                if angle_deg > angle_tolerance_deg:
                    continue

                combined_points = np.vstack([seg1['points'], seg2['points']])
                center, direction, error = self._fit_line(combined_points)

                if error >= max_merge_error:
                    continue

                merged_line = {
                    'type': 'line',
                    'points': combined_points,
                    'center': center,
                    'direction': direction,
                    'error': error,
                }

                segments = segments[:i] + [merged_line] + segments[i+2:]
                merge_found = True
                break

            if not merge_found:
                break

        return segments

    def _merge_small_lines_into_arcs(
        self,
        segments: List[Dict],
        min_line_size: int = 70,
        max_merge_error: float = 0.01,
    ) -> List[Dict]:
        """Merge small lines into adjacent arcs.

        Args:
            segments: List of geometry dictionaries
            min_line_size: Minimum line segment size to keep separate
            max_merge_error: Maximum fit error to allow merge

        Returns:
            List of merged geometry dictionaries
        """

        changed = True
        iteration = 0

        while changed and iteration < 10:
            changed = False
            iteration += 1

            new_segments = []
            absorbed = set()

            for i, segment in enumerate(segments):
                if i in absorbed:
                    continue

                if segment['type'] != 'arc':
                    new_segments.append(segment)
                    continue

                left_indices = []
                right_indices = []

                j = i - 1
                while j >= 0 and j not in absorbed:
                    if segments[j]['type'] == 'line' and len(segments[j]['points']) < min_line_size:
                        left_indices.insert(0, j)
                        j -= 1
                    elif segments[j]['type'] == 'arc':
                        left_indices.insert(0, j)
                        break
                    else:
                        break

                j = i + 1
                while j < len(segments) and j not in absorbed:
                    if segments[j]['type'] == 'line' and len(segments[j]['points']) < min_line_size:
                        right_indices.append(j)
                        j += 1
                    elif segments[j]['type'] == 'arc':
                        right_indices.append(j)
                        break
                    else:
                        break

                absorb_indices = left_indices + [i] + right_indices

                if len(absorb_indices) > 1:
                    combined_points = np.vstack([segments[idx]['points'] for idx in absorb_indices])
                    center, radius, error = self._fit_circle(combined_points)

                    if radius is not None and error < max_merge_error:
                        merged_segment = {
                            'type': 'arc',
                            'points': combined_points,
                            'center': center,
                            'radius': radius,
                            'error': error,
                        }
                        new_segments.append(merged_segment)
                        absorbed.update(absorb_indices)
                        changed = True
                    else:
                        new_segments.append(segment)
                else:
                    new_segments.append(segment)

            segments = new_segments

        return segments

    def _remove_outliers(
        self,
        points: np.ndarray,
        std_threshold: float = 5.0,
    ) -> np.ndarray:
        """Remove outlier points.

        Args:
            points: Input path points (N, 3)
            std_threshold: Number of standard deviations for outlier detection

        Returns:
            Cleaned point array with outliers removed
        """

        if len(points) < 3:
            return points

        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)

        if len(distances) < 2:
            return points

        mean_dist = np.mean(distances)
        std_dist = np.std(distances)

        if std_dist < 1e-10:
            return points

        outlier_mask = distances > (mean_dist + std_threshold * std_dist)
        keep_mask = np.ones(len(points), dtype=bool)
        keep_mask[:-1] = ~outlier_mask

        return points[keep_mask]

    def _apply_spline_smoothing(
        self,
        points: np.ndarray,
        is_closed: bool = False,
        num_points: int = 100,
        smoothing_factor: float = 0.0,
    ) -> np.ndarray:
        """Apply B-spline smoothing.

        Args:
            points: Input path points (N, 3)
            is_closed: True for closed loops
            num_points: Number of points in smoothed output
            smoothing_factor: Spline smoothing factor (0 = interpolation)

        Returns:
            Smoothed point array (num_points, 3)
        """

        vertices = np.asarray(points, dtype=np.float64)

        if len(vertices) < 4:
            return vertices

        x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]

        try:
            tck, u = splprep([x, y, z], s=smoothing_factor, per=1 if is_closed else 0)
        except Exception:
            return vertices

        u_new = np.linspace(0, 1, num_points)
        x_smooth, y_smooth, z_smooth = splev(u_new, tck)

        return np.column_stack([x_smooth, y_smooth, z_smooth])

    def _classify_segment(
        self,
        points: np.ndarray,
        tie_error_threshold: float = 0.0001,
        max_realistic_radius: float = 2.0,
    ) -> tuple[str, dict]:
        """Classify segment as line or arc.

        Args:
            points: Segment points (N, 3)
            tie_error_threshold: Error threshold for tie-breaking
            max_realistic_radius: Maximum realistic arc radius

        Returns:
            Tuple of (geometry_type, parameters_dict)
        """

        if len(points) < 3:
            center, direction, error = self._fit_line(points)
            return 'line', {
                'center': center,
                'direction': direction,
                'error': error,
            }

        line_center, line_direction, line_error = self._fit_line(points)
        circle_center, radius, circle_error = self._fit_circle(points)

        if line_error < tie_error_threshold and circle_error < tie_error_threshold:
            if radius is not None and radius > max_realistic_radius:
                return 'line', {
                    'center': line_center,
                    'direction': line_direction,
                    'error': line_error,
                }
            else:
                return 'arc', {
                    'center': circle_center,
                    'radius': radius,
                    'error': circle_error,
                }

        if radius is not None and circle_error < line_error:
            return 'arc', {
                'center': circle_center,
                'radius': radius,
                'error': circle_error,
            }
        else:
            return 'line', {
                'center': line_center,
                'direction': line_direction,
                'error': line_error,
            }

    def _fit_line(
        self,
        points: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Fit 3D line using PCA.

        Args:
            points: Input points (N, 3)

        Returns:
            Tuple of (center, direction, error)
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
        error = np.sum(distances**2)

        return centroid, direction, error

    def _fit_circle(
        self,
        points: np.ndarray,
    ) -> tuple[np.ndarray, float, float]:
        """Fit circle using least squares.

        Args:
            points: Input points (N, 3)

        Returns:
            Tuple of (center, radius, error)
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
            error = np.sum(result.fun**2)

            center_2d = np.array([cx_2d, cy_2d])
            center_3d = pca.inverse_transform(center_2d.reshape(1, -1))[0] + centroid

            return center_3d, abs(radius), error

        except Exception:
            return None, None, float('inf')
