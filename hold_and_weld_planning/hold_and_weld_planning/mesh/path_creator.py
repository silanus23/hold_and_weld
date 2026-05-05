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

import logging
from typing import Any, Dict, List, Optional

import numpy as np
from scipy.interpolate import splev, splprep
from scipy.optimize import least_squares
from sklearn.decomposition import PCA

logger = logging.getLogger(__name__)


class PathCreator:
    """Smooth raw vertex paths and detect geometric primitives.

    Configuration Parameters (passed via config dict to process_path):
        outlier_std_threshold: Standard deviations for outlier detection (default: 5.0)
        min_sub_path_length: Min points per sub-path after corner split (default: 10)
        min_points_for_corner_detection: Min points to attempt corner detection (default: 20)
        corner_min_angle: Min angle change for corner detection in degrees (default: 25)
        corner_angle_window: Window size for angle-based detection (default: 10)
        corner_curvature_window: Window size for curvature-based detection (default: 10)
        corner_curvature_threshold: Normalized curvature threshold 0-1 (default: 0.8)
        corner_min_agreement: Min methods that must agree on corner (default: 1)
        corner_tolerance: Max distance between corners to group (default: 5)
        corner_filter_window: Window for local maxima filtering (default: 5)
        arc_radius_tolerance: Max radius difference to merge arcs in meters (default: 0.05)
        arc_merge_num_points: Points for smoothed merged arc (default: 100)
        line_angle_tolerance_deg: Max angle diff to merge lines in degrees (default: 10.0)
        line_merge_max_error: Max fit error to allow line merge (default: 0.001)
        line_merge_max_iterations: Max iterations for line merging (default: 20)
        min_line_size: Min line points to keep separate from arcs (default: 70)
        line_absorption_max_error: Max error to merge small lines into arcs (default: 0.01)
        line_absorption_max_iterations: Max iterations for absorption (default: 10)
        first_last_angle_tolerance: Angle tolerance for first/last merge (default: 10)
        first_last_radius_tolerance: Radius tolerance for first/last merge (default: 0.05)
        first_last_gap_threshold: Gap threshold for first/last merge in meters (default: 0.01)
    """

    def process_path(
        self,
        points: np.ndarray,
        is_closed: bool = False,
        num_points: int = 100,
        spline_smoothing_factor: float = 0.0,
        config: Optional[Dict[str, Any]] = None,
    ) -> List[Dict]:
        """Process raw path: detect corners, smooth with splines, classify geometry, merge.

        Detects corners, smooths with splines, classifies geometry, and merges primitives.

        Args:
            points: Raw path points (N, 3)
            is_closed: Whether path forms closed loop
            num_points: Points per smoothed segment
            spline_smoothing_factor: B-spline smoothing factor
                (0 = interpolation, higher = smoother)
            config: Optional dict with tuning parameters (see class docstring for available keys)

        Returns:
            List of geometry dicts with 'type' ('line'/'arc'), 'points', 'center',
            and fit parameters
        """
        cfg = config or {}

        std_threshold = cfg.get('outlier_std_threshold', 5.0)

        min_sub_path_length = cfg.get('min_sub_path_length', 10)
        min_points_for_corners = cfg.get('min_points_for_corner_detection', 20)
        corner_min_angle = cfg.get('corner_min_angle', 25)
        corner_angle_window = cfg.get('corner_angle_window', 10)
        corner_curvature_window = cfg.get('corner_curvature_window', 10)
        corner_curvature_threshold = cfg.get('corner_curvature_threshold', 0.8)
        corner_min_agreement = cfg.get('corner_min_agreement', 1)
        corner_tolerance = cfg.get('corner_tolerance', 5)
        corner_filter_window = cfg.get('corner_filter_window', 5)

        arc_radius_tolerance = cfg.get('arc_radius_tolerance', 0.05)
        arc_merge_num_points = cfg.get('arc_merge_num_points', 100)
        line_angle_tolerance_deg = cfg.get('line_angle_tolerance_deg', 10.0)
        line_merge_max_error = cfg.get('line_merge_max_error', 0.001)
        line_merge_max_iterations = cfg.get('line_merge_max_iterations', 20)

        min_line_size = cfg.get('min_line_size', 70)
        line_absorption_max_error = cfg.get('line_absorption_max_error', 0.01)
        line_absorption_max_iterations = cfg.get('line_absorption_max_iterations', 10)

        first_last_angle_tolerance = cfg.get('first_last_angle_tolerance', 10)
        first_last_radius_tolerance = cfg.get('first_last_radius_tolerance', 0.05)
        first_last_gap_threshold = cfg.get('first_last_gap_threshold', 0.01)

        logger.debug(f'Processing path with {len(points)} points (closed={is_closed})')

        points_cleaned = self._remove_outliers(points, std_threshold=std_threshold)
        logger.debug(f'After outlier removal: {len(points_cleaned)} points')

        sub_paths = self._split_path_at_corners(
            points_cleaned,
            min_sub_path_length=min_sub_path_length,
            min_points_for_corners=min_points_for_corners,
            corner_min_angle=corner_min_angle,
            corner_angle_window=corner_angle_window,
            corner_curvature_window=corner_curvature_window,
            corner_curvature_threshold=corner_curvature_threshold,
            corner_min_agreement=corner_min_agreement,
            corner_tolerance=corner_tolerance,
            corner_filter_window=corner_filter_window,
        )
        logger.debug(f'Split into {len(sub_paths)} sub-path(s) at detected corners')

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

        if len(segments_with_raw) > 1:
            first_geom = segments_with_raw[0][0]
            last_geom = segments_with_raw[-1][0]

            if self._should_merge_first_last(
                sub_paths, first_geom, last_geom,
                angle_tolerance=first_last_angle_tolerance,
                radius_tolerance=first_last_radius_tolerance,
                gap_threshold=first_last_gap_threshold,
            ):
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

        result = self._merge_consecutive_arcs(
            segments_with_raw,
            radius_tolerance=arc_radius_tolerance,
            num_points=arc_merge_num_points,
        )
        result = self._merge_consecutive_lines(
            result,
            angle_tolerance_deg=line_angle_tolerance_deg,
            max_merge_error=line_merge_max_error,
            max_iterations=line_merge_max_iterations,
        )
        result = self._merge_small_lines_into_arcs(
            result,
            min_line_size=min_line_size,
            max_merge_error=line_absorption_max_error,
            max_iterations=line_absorption_max_iterations,
        )

        logger.debug(f'Path processing complete: {len(result)} final segment(s)')
        return result

    def _should_merge_first_last(
        self,
        sub_paths: List[np.ndarray],
        first_geom: Dict,
        last_geom: Dict,
        angle_tolerance: float = 10,
        radius_tolerance: float = 0.05,
        gap_threshold: float = 0.01,
    ) -> bool:
        """Check if first and last segments should merge (closed path split arbitrarily)."""
        if len(sub_paths) <= 1:
            return False

        # TODO(@silanus23): Test this
        # Heuristic check: merges first/last segments for closed paths that were
        # split arbitrarily at the seam. Relies on geometry type, orientation, and
        # endpoint proximity. Treat results with caution on atypical geometries.
        if first_geom['type'] != last_geom['type']:
            return False

        if first_geom['type'] == 'line':
            dir1 = first_geom['direction']
            dir2 = last_geom['direction']
            cos_angle = abs(np.dot(dir1, dir2))
            angle_deg = np.degrees(np.arccos(np.clip(cos_angle, 0, 1)))

            if angle_deg > angle_tolerance:
                return False

        elif first_geom['type'] == 'arc':
            r1 = first_geom['radius']
            r2 = last_geom['radius']

            if abs(r1 - r2) > radius_tolerance:
                return False

        first_path = sub_paths[0]
        last_path = sub_paths[-1]

        gap = np.linalg.norm(last_path[-1] - first_path[0])

        if gap > gap_threshold:
            return False

        return True

    def _split_path_at_corners(
        self,
        points: np.ndarray,
        min_sub_path_length: int = 10,
        min_points_for_corners: int = 20,
        corner_min_angle: float = 25,
        corner_angle_window: int = 10,
        corner_curvature_window: int = 10,
        corner_curvature_threshold: float = 0.8,
        corner_min_agreement: int = 1,
        corner_tolerance: int = 5,
        corner_filter_window: int = 5,
    ) -> List[np.ndarray]:
        """Detect corners and split path into sub-paths."""
        if len(points) < min_points_for_corners:
            return [points]

        angle_corners = self._detect_corners_by_angle(
            points,
            min_angle=corner_min_angle,
            window=corner_angle_window,
            filter_window=corner_filter_window,
        )
        curvature_corners = self._detect_corners_by_curvature(
            points,
            window=corner_curvature_window,
            threshold=corner_curvature_threshold,
        )

        corner_indices = self._combine_corner_detections(
            [angle_corners, curvature_corners],
            min_agreement=corner_min_agreement,
            tolerance=corner_tolerance,
        )

        logger.debug(f'Detected {len(corner_indices)} corner(s) in path')

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
                # Merge tiny sub-path with previous to avoid degenerate segments
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
        filter_window: int = 5,
    ) -> List[int]:
        """Detect corners by angle change between before/after vectors."""
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f'Points must be (N, 3) array, got shape {points.shape}')

        if window <= 0:
            raise ValueError(f'Window must be positive, got {window}')

        if not (0 < min_angle <= 180):
            raise ValueError(f'min_angle must be in (0, 180], got {min_angle}')

        if len(points) < 2 * window:
            logger.debug(
                f'Path too short ({len(points)} points) for corner detection with window={window}'
            )
            return []

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

        logger.info(f'Angle-based detection found {len(corners)} candidate(s) before filtering')
        return self._filter_to_local_maxima(corners, points, window=filter_window)

    def _detect_corners_by_curvature(
        self,
        points: np.ndarray,
        window: int = 10,
        threshold: float = 0.8,
    ) -> List[int]:
        """Detect corners by discrete curvature magnitude."""
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f'Points must be (N, 3) array, got shape {points.shape}')

        if window <= 0:
            raise ValueError(f'Window must be positive, got {window}')

        if not (0 < threshold <= 1):
            raise ValueError(f'Threshold must be in (0, 1], got {threshold}')

        if len(points) < window * 2:
            logger.debug(
                f'Path too short ({len(points)} points) for curvature'
                f' detection with window={window}'
            )
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
            logger.debug('No significant curvature variation detected')
            return []

        max_curvature = np.max(curvatures)
        logger.debug(f'Max curvature: {max_curvature:.6f}')
        normalized = curvatures / max_curvature

        corners = []
        for i in range(window, len(normalized) - window):
            if normalized[i] > threshold:
                local_start = max(0, i - 5)
                local_end = min(len(normalized), i + 6)
                if normalized[i] == np.max(normalized[local_start:local_end]):
                    corners.append(i)

        logger.debug(f'Curvature-based detection found {len(corners)} corner(s)')
        return corners

    def _combine_corner_detections(
        self,
        corner_lists: List[List[int]],
        min_agreement: int = 1,
        tolerance: int = 5,
    ) -> List[int]:
        """Combine corner detections from multiple methods using spatial grouping."""
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
                # Use median as robust central estimate - less sensitive to outliers than mean
                # when multiple detection methods report slightly different indices for same corner
                verified.append(int(np.median(group)))

        return sorted(verified)

    def _filter_to_local_maxima(
        self,
        indices: List[int],
        points: np.ndarray,
        window: int = 5,
    ) -> List[int]:
        """Filter candidate corners to local maxima of angle change."""
        if not indices:
            return []

        if window <= 0:
            raise ValueError(f'Window must be positive, got {window}')

        # Use larger window for stable angle measurement
        angle_window = max(window * 2, 10)

        angles = {}
        for idx in indices:
            if idx < angle_window or idx >= len(points) - angle_window:
                continue

            v1 = points[idx] - points[idx - angle_window]
            v2 = points[idx + angle_window] - points[idx]

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
        num_points: int = 100,
    ) -> List[Dict]:
        """Merge consecutive arcs with similar radii."""
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
                # Refit circle to combined points for best accuracy
                center, radius, error = self._fit_circle(combined_raw)

                smoothed = self._apply_spline_smoothing(
                    combined_raw, is_closed=False, num_points=num_points
                )

                merged_geom = {
                    'type': 'arc',
                    'points': smoothed,
                    'center': center,
                    'radius': radius,
                    'error': error,
                }

                logger.debug(
                    f'Merged {len(arc_group_geoms)} consecutive arcs'
                    f' into one (radius={radius:.4f}m)'
                )
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
        max_iterations: int = 20,
    ) -> List[Dict]:
        """Merge consecutive collinear lines iteratively."""
        iteration = 0

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

                # Reject merge if fit quality degrades
                if error >= max_merge_error:
                    continue

                merged_line = {
                    'type': 'line',
                    'points': combined_points,
                    'center': center,
                    'direction': direction,
                    'error': error,
                }

                logger.debug(
                    f'Merged 2 collinear lines (angle diff={angle_deg:.2f}°, error={error:.6f})'
                )
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
        max_iterations: int = 10,
    ) -> List[Dict]:
        """Absorb small line segments into adjacent arcs."""
        changed = True
        iteration = 0

        while changed and iteration < max_iterations:
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
                    if (segments[j]['type'] == 'line' and
                            len(segments[j]['points']) < min_line_size):
                        left_indices.insert(0, j)
                        j -= 1
                    elif segments[j]['type'] == 'arc':
                        left_indices.insert(0, j)
                        break
                    else:
                        break

                j = i + 1
                while j < len(segments) and j not in absorbed:
                    if (segments[j]['type'] == 'line' and len(segments[j]['points']) <
                            min_line_size):
                        right_indices.append(j)
                        j += 1
                    elif segments[j]['type'] == 'arc':
                        right_indices.append(j)
                        break
                    else:
                        break

                absorb_indices = left_indices + [i] + right_indices

                if len(absorb_indices) > 1:
                    combined_points = np.vstack([segments[idx]['points'] for
                                                idx in absorb_indices])
                    # Refit circle to combined geometry
                    center, radius, error = self._fit_circle(combined_points)

                    if radius is not None and error < max_merge_error:
                        merged_segment = {
                            'type': 'arc',
                            'points': combined_points,
                            'center': center,
                            'radius': radius,
                            'error': error,
                        }
                        logger.debug(
                            f'Merged {len(absorb_indices)} segment(s)'
                            f' into arc (radius={radius:.4f}m)'
                        )
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
        """Remove outlier points using statistical distance threshold.

        A conservative method to handle big problems.
        """
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f'Points must be (N, 3) array, got shape {points.shape}')

        if std_threshold <= 0:
            raise ValueError(f'std_threshold must be positive, got {std_threshold}')

        if len(points) < 3:
            return points

        distances = np.linalg.norm(np.diff(points, axis=0), axis=1)

        if len(distances) < 2:
            return points

        mean_dist = np.mean(distances)
        std_dist = np.std(distances)

        if std_dist < 1e-10:
            logger.debug('Outlier removal skipped (no significant point spacing variation)')
            return points

        outlier_mask = distances > (mean_dist + std_threshold * std_dist)
        keep_mask = np.ones(len(points), dtype=bool)
        # distances has N-1 entries (np.diff), so [:-1] maps each distance back to its
        # source point. ~ inverts the boolean mask: True = outlier becomes False = remove.
        keep_mask[:-1] = ~outlier_mask

        num_removed = np.sum(~keep_mask)
        if num_removed > 0:
            logger.debug(f'Removed {num_removed} outlier point(s)')

        return points[keep_mask]

    def _apply_spline_smoothing(
        self,
        points: np.ndarray,
        is_closed: bool = False,
        num_points: int = 100,
        smoothing_factor: float = 0.0,
    ) -> np.ndarray:
        """Apply B-spline smoothing to path points."""
        vertices = np.asarray(points, dtype=np.float64)

        if len(vertices) < 4:
            return vertices

        x, y, z = vertices[:, 0], vertices[:, 1], vertices[:, 2]

        try:
            tck, u = splprep([x, y, z], s=smoothing_factor, per=1 if is_closed else 0)
        except Exception as e:
            logger.warning(f'Spline fitting failed: {e}, returning original points')
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
        """Classify segment as line or arc by comparing fit errors."""
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
            # Both fits are excellent — distinguish by radius and relative error.
            # Large radius: arc is geometrically indistinguishable from a line, prefer line.
            if radius is not None and radius > max_realistic_radius:
                logger.debug(
                    f'Tie-break: radius={radius:.3f}m exceeds max_realistic_radius,'
                    ' classifying as line'
                )
                return 'line', {
                    'center': line_center,
                    'direction': line_direction,
                    'error': line_error,
                }
            # When both errors are tiny, prefer line unless circle fit is strictly better.
            # Avoids misclassifying near-straight segments as arcs due to numerical noise.
            elif line_error <= circle_error:
                logger.debug(
                    f'Tie-break: line_error={line_error:.6f} <= circle_error={circle_error:.6f},'
                    ' classifying as line'
                )
                return 'line', {
                    'center': line_center,
                    'direction': line_direction,
                    'error': line_error,
                }
            else:
                logger.debug(
                    f'Tie-break: circle_error={circle_error:.6f} < line_error={line_error:.6f},'
                    ' classifying as arc'
                )
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
        """Fit 3D line using PCA, return center, direction, and error."""
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
        """Fit circle using least squares in PCA-projected 2D plane."""
        if len(points) < 3:
            return None, None, float('inf')

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        # Project to 2D via PCA — arc points are planar, PCA finds that plane.
        pca = PCA(n_components=2)
        points_2d = pca.fit_transform(centered)

        center_2d_guess = np.mean(points_2d, axis=0)
        radius_guess = np.mean(np.linalg.norm(points_2d - center_2d_guess, axis=1))

        # Nonlinear least squares: minimize deviation from a constant radius.
        # residuals() is a closure over points_2d — scipy calls it repeatedly,
        # adjusting [cx, cy, r] until sum(residuals**2) is minimized.
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
            # Map 2D fitted center back to 3D: undo PCA projection then undo centring.
            center_3d = pca.inverse_transform(center_2d.reshape(1, -1))[0] + centroid

            return center_3d, abs(radius), error

        except Exception as e:
            logger.warning(f'Circle fit failed: {e}')
            return None, None, float('inf')
