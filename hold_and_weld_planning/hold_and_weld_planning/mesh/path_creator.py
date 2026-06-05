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

"""Classify ordered SeamPoints into geometric segments wrapped in Seam objects."""

import logging
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
from sklearn.decomposition import PCA

from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
from ..core.ptp_segment import PtPSegment
from ..core.seam import Seam
from .seam_extractor import SeamPoint

logger = logging.getLogger(__name__)


class PathCreator:
    """Classify ordered SeamPoints into geometric segments and wrap in Seam objects.

    Configuration Parameters:
        outlier_std_threshold: Std-dev multiplier above mean gap to flag outlier (default 5.0).
        window_size:           Seed window size in points for classification (default 20).
        line_error_threshold:  Normalized line fit error threshold (default 1e-4).
        arc_error_threshold:   Normalized arc fit error threshold (default 1e-4).
        max_line_length:       Maximum line segment arc length in meters (default 0.5).
        max_arc_length:        Maximum arc segment arc length in meters (default 0.5).
        max_ptp_length:        Maximum PtP segment arc length in meters (default 0.1).
    """

    _DEFAULTS = {
        'outlier_std_threshold': 5.0,
        'window_size': 20,
        'line_error_threshold': 1e-4,
        'arc_error_threshold': 1e-4,
        'max_line_length': 0.5,
        'max_arc_length': 0.5,
        'max_ptp_length': 0.1,
    }

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        """Args:
            config: Optional configuration dict (see class docstring for keys).
        """
        self._init_config: Dict[str, Any] = config or {}
        self._apply_config(None)

    def _apply_config(self, config: Optional[Dict[str, Any]]) -> None:
        """Merge per-call config > init config > defaults into instance attributes."""
        cfg = {**self._init_config, **(config or {})}

        self.outlier_std_threshold = cfg.get(
            'outlier_std_threshold', self._DEFAULTS['outlier_std_threshold']
        )
        self.window_size = cfg.get('window_size', self._DEFAULTS['window_size'])
        self.line_error_threshold = cfg.get(
            'line_error_threshold', self._DEFAULTS['line_error_threshold']
        )
        self.arc_error_threshold = cfg.get(
            'arc_error_threshold', self._DEFAULTS['arc_error_threshold']
        )
        self.max_line_length = cfg.get('max_line_length', self._DEFAULTS['max_line_length'])
        self.max_arc_length = cfg.get('max_arc_length', self._DEFAULTS['max_arc_length'])
        self.max_ptp_length = cfg.get('max_ptp_length', self._DEFAULTS['max_ptp_length'])

    def process_path(
        self,
        seam_points: List[SeamPoint],
        config: Optional[Dict[str, Any]] = None,
    ) -> List[Seam]:
        """Process ordered SeamPoints into classified Seam objects.

        Args:
            seam_points: Ordered SeamPoint list from SeamExtractor.
            config:      Optional per-call config overriding construction config.

        Returns:
            List of Seam objects. Empty if fewer than 2 valid points after outlier removal.
        """
        if config is not None:
            self._apply_config(config)

        if len(seam_points) < 2:
            logger.warning(f'Too few SeamPoints to process: {len(seam_points)}')
            return []

        cleaned = self._remove_outliers(seam_points)
        logger.debug(f'After outlier removal: {len(cleaned)}/{len(seam_points)} points')

        if len(cleaned) < 2:
            return []

        sublists = self._hard_split_on_contact_type(cleaned)
        logger.debug(f'Hard split produced {len(sublists)} sublist(s)')

        seams = []

        for sublist_idx, sublist in enumerate(sublists):
            positions = np.array([sp.position for sp in sublist])

            segments = self._classify_subpath(positions)
            logger.debug(
                f'Sublist {sublist_idx}: {len(segments)} segment(s) from '
                f'{len(positions)} points'
            )

            for seg_points, seg_type in segments:
                split_segs = self._split_by_length(seg_points, seg_type)

                for split_pts in split_segs:
                    seam = self._wrap_in_seam(split_pts, seg_type, sublist)
                    if seam is not None:
                        seams.append(seam)

        logger.info(f'PathCreator produced {len(seams)} Seam object(s)')
        return seams

    def _remove_outliers(
        self,
        seam_points: List[SeamPoint],
    ) -> List[SeamPoint]:
        """Remove positional outliers by inter-point distance statistics.

        Flags interior points only when both incident gaps are oversized.
        Endpoint flags require only their single incident gap to be oversized.
        """
        if len(seam_points) < 3:
            return seam_points

        positions = np.array([sp.position for sp in seam_points])
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)

        if len(distances) < 2:
            return seam_points

        mean_dist = np.mean(distances)
        std_dist = np.std(distances)

        if std_dist < 1e-10:
            return seam_points

        threshold = mean_dist + self.outlier_std_threshold * std_dist
        big = distances > threshold

        n = len(seam_points)
        keep_mask = np.ones(n, dtype=bool)

        for i in range(n):
            if i == 0:
                if big[0]:
                    keep_mask[i] = False
            elif i == n - 1:
                if big[n - 2]:
                    keep_mask[i] = False
            else:
                if big[i - 1] and big[i]:
                    keep_mask[i] = False

        filtered = [sp for sp, keep in zip(seam_points, keep_mask) if keep]

        removed = len(seam_points) - len(filtered)
        if removed > 0:
            logger.debug(f'Removed {removed} outlier point(s)')

        return filtered

    def _hard_split_on_contact_type(
        self,
        seam_points: List[SeamPoint],
    ) -> List[List[SeamPoint]]:
        """Split at (on_edge_1, on_edge_2) contact type changes; discard sublists < 2 points."""
        if not seam_points:
            return []

        sublists = []
        current = [seam_points[0]]
        current_type = (seam_points[0].on_edge_1, seam_points[0].on_edge_2)

        for sp in seam_points[1:]:
            sp_type = (sp.on_edge_1, sp.on_edge_2)
            if sp_type != current_type:
                if len(current) >= 2:
                    sublists.append(current)
                current = [sp]
                current_type = sp_type
            else:
                current.append(sp)

        if len(current) >= 2:
            sublists.append(current)

        return sublists

    def _classify_subpath(
        self,
        positions: NDArray,
    ) -> List[Tuple[NDArray, str]]:
        """Classify positions into (line/arc/ptp) segments using a look-ahead consumer.

        Seeds a window_size window, grows greedily while fit error stays below threshold,
        then commits. PTP regions advance one point at a time until a valid seed is found.
        """
        n = len(positions)
        segments = []
        k = 0

        while k < n:
            remaining = n - k

            if remaining < 4:
                segments.append((positions[k:n], 'ptp'))
                break

            end = min(k + self.window_size, n)
            seed_pts = positions[k:end]

            line_err = self._normalized_line_error(seed_pts)

            if line_err < self.line_error_threshold:
                seed_type = 'line'
            else:
                arc_err = self._normalized_arc_error(seed_pts)
                if arc_err is not None and arc_err < self.arc_error_threshold:
                    seed_type = 'arc'
                else:
                    seed_type = 'ptp'

            if seed_type == 'ptp':
                m = k + 1
                while m < n:
                    next_end = min(m + self.window_size, n)
                    if next_end - m >= 4:
                        next_pts = positions[m:next_end]
                        if self._normalized_line_error(next_pts) < self.line_error_threshold:
                            break
                        next_arc_err = self._normalized_arc_error(next_pts)
                        if next_arc_err is not None and next_arc_err < self.arc_error_threshold:
                            break
                    m += 1

                segments.append((positions[k:m], 'ptp'))
                k = m

            else:
                m = end

                while m < n:
                    test_pts = positions[k : m + 1]

                    if seed_type == 'line':
                        err = self._normalized_line_error(test_pts)
                        if err >= self.line_error_threshold:
                            break
                    else:  # arc
                        err = self._normalized_arc_error(test_pts)
                        if err is None or err >= self.arc_error_threshold:
                            break

                    m += 1

                segments.append((positions[k:m], seed_type))
                k = m

        return segments

    def _split_by_length(
        self,
        points: NDArray,
        seg_type: str,
    ) -> List[NDArray]:
        """Split segment into equal subsegments if total arc length exceeds the type max."""
        if seg_type == 'line':
            max_len = self.max_line_length
        elif seg_type == 'arc':
            max_len = self.max_arc_length
        else:
            max_len = self.max_ptp_length

        if len(points) < 2:
            return [points]

        segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        total_length = float(np.sum(segment_lengths))

        if total_length <= max_len:
            return [points]

        n_splits = int(np.ceil(total_length / max_len))
        target_length = total_length / n_splits

        result = []
        current_start = 0
        cumulative = 0.0

        for i in range(1, len(points)):
            cumulative += segment_lengths[i - 1]

            if cumulative >= target_length and i < len(points) - 1:
                result.append(points[current_start : i + 1])
                current_start = i
                cumulative = 0.0

        last = points[current_start:]
        if len(last) >= 2:
            result.append(last)
        elif result:
            result[-1] = np.vstack([result[-1], last])

        return [r for r in result if len(r) >= 2]

    def _wrap_in_seam(
        self,
        points: NDArray,
        seg_type: str,
        seam_points_subset: List[SeamPoint],
    ) -> Optional[Seam]:
        """Wrap positions into a Seam with nearest-neighbour normals and contact metadata."""
        if len(points) < 2:
            return None

        subset_positions = np.array([sp.position for sp in seam_points_subset])
        tree = KDTree(subset_positions)

        normals_main = []
        normals_secondary = []
        on_edge_1_vals = []
        on_edge_2_vals = []

        for pt in points:
            _, idx = tree.query(pt)
            sp = seam_points_subset[idx]
            normals_main.append(sp.normal_main)
            normals_secondary.append(sp.normal_secondary)
            on_edge_1_vals.append(sp.on_edge_1)
            on_edge_2_vals.append(sp.on_edge_2)

        normals_main = np.array(normals_main)
        normals_secondary = np.array(normals_secondary)

        on_edge_1 = on_edge_1_vals[0]
        on_edge_2 = on_edge_2_vals[0]
        is_edge_joint = on_edge_1 and on_edge_2

        try:
            if seg_type == 'line':
                segment = LineSegment(start=points[0], end=points[-1])
                seam = Seam(line_segment=segment)

            elif seg_type == 'arc':
                result = self._fit_circle_taubin(points)
                if result is None:
                    logger.warning(
                        'Arc fit failed in _wrap_in_seam — falling back to PtPSegment'
                    )
                    segment = PtPSegment(points=points)
                    seam = Seam(ptp_segment=segment)
                    seg_type = 'ptp'
                else:
                    center, radius, _ = result
                    segment = ArcSegment(points=points, center=center, radius=radius)
                    seam = Seam(arc_segment=segment)

            else:  # ptp
                segment = PtPSegment(points=points)
                seam = Seam(ptp_segment=segment)

        except Exception as e:
            logger.warning(f'Segment construction failed in _wrap_in_seam: {e}')
            return None

        seam.config['is_edge_joint'] = is_edge_joint
        seam.config['on_edge_1'] = on_edge_1
        seam.config['on_edge_2'] = on_edge_2
        seam.config['geometry_type'] = seg_type
        seam.config['smoothed_points'] = points
        seam.config['normals_main'] = normals_main
        seam.config['normals_secondary'] = normals_secondary

        return seam

    def _normalized_line_error(self, points: NDArray) -> float:
        """Sum-squared line fit error divided by point count."""
        _, _, error = self._fit_line(points)
        return error / len(points)

    def _normalized_arc_error(self, points: NDArray) -> Optional[float]:
        """Sum-squared arc fit error divided by point count, or None on degenerate geometry."""
        result = self._fit_circle_taubin(points)
        if result is None:
            return None
        _, _, error = result
        return error / len(points)

    def _fit_line(
        self,
        points: NDArray,
    ) -> Tuple[NDArray, NDArray, float]:
        """Fit 3D line via PCA; return (centroid, direction, sum_squared_perpendicular_error)."""
        if len(points) < 2:
            return points[0].copy(), np.array([1.0, 0.0, 0.0]), 0.0

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        pca = PCA(n_components=1)
        pca.fit(centered)
        direction = pca.components_[0]

        t = np.dot(centered, direction)
        projected = t[:, np.newaxis] * direction
        residuals = centered - projected

        error = float(np.sum(np.linalg.norm(residuals, axis=1) ** 2))
        return centroid, direction, error

    def _fit_circle_taubin(
        self,
        points: NDArray,
    ) -> Optional[Tuple[NDArray, float, float]]:
        """Algebraic circle fit via linear least squares in PCA-projected 2D plane.

        Args:
            points: Input positions (N, 3), N >= 3.

        Returns:
            (center_3d, radius, sum_squared_radial_error) or None if fit fails.
        """
        if len(points) < 3:
            return None

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        pca = PCA(n_components=2)
        pts_2d = pca.fit_transform(centered)

        xi = pts_2d[:, 0]
        yi = pts_2d[:, 1]
        zi = xi ** 2 + yi ** 2

        A = np.column_stack([xi, yi, np.ones(len(pts_2d))])

        try:
            params, _, _, _ = np.linalg.lstsq(A, zi, rcond=None)
        except Exception as e:
            logger.debug(f'Circle lstsq failed: {e}')
            return None

        a = params[0] / 2.0
        b = params[1] / 2.0
        c = params[2]

        r_sq = a ** 2 + b ** 2 + c
        if r_sq <= 0:
            return None

        radius = float(np.sqrt(r_sq))
        center_2d = np.array([a, b])

        center_3d = pca.inverse_transform(center_2d.reshape(1, -1))[0] + centroid

        dists = np.sqrt((xi - a) ** 2 + (yi - b) ** 2)
        error = float(np.sum((dists - radius) ** 2))

        return center_3d, radius, error
