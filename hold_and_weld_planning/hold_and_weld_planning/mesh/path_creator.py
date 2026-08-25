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

"""PathCreator - Classify ordered SeamPoints into geometric segments.

A run is a LINE when a straight line holds the process path tolerance, an ARC
when a circle holds the stricter arc tolerance over a minimum angle, and PTP
otherwise. Line wins ties; forcing a near-circle into an arc leaves the seam,
while demoting a true arc to PTP only densifies waypoints.
"""

import logging
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
from ..core.ptp_segment import PtPSegment
from ..core.seam import Seam
from .seam_point import SeamPoint

logger = logging.getLogger(__name__)


class PathCreator:
    """Classify ordered SeamPoints into segments wrapped in Seam objects.

    Args:
        config: Configuration dict; see PARAMS.md.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        """Initialize the classifier; see the class docstring for arguments."""
        self._init_config: Dict[str, Any] = config or {}
        self._apply_config(None)

    def _apply_config(self, config: Optional[Dict[str, Any]]) -> None:
        """Merge per-call config > init config > defaults into attributes."""
        cfg = {**self._init_config, **(config or {})}

        tolerance = float(cfg.get('path_tolerance_mm', 1.0)) * 1e-3

        self.tolerance = tolerance
        self.arc_tolerance = float(cfg.get('arc_strictness', 0.5)) * tolerance
        self.min_arc_angle = np.radians(
            float(cfg.get('min_arc_angle_deg', 15.0)))
        self.max_line_length = float(cfg.get('max_line_length', 0.5))
        self.max_arc_length = float(cfg.get('max_arc_length', 0.5))
        self.max_ptp_length = float(cfg.get('max_ptp_length', 0.1))
        self.min_fit_points = int(cfg.get('min_fit_points', 4))
        self.arc_gain = float(cfg.get('arc_gain', 1.5))

        # Not its own key: a run too short to carry two weld poses is not a
        # weld segment, so the process spacing is the threshold.
        self.min_contact_run = float(
            cfg.get('waypoint_spacing_mm', 10.0)) * 1e-3

        self._validate_config(cfg)

    def _validate_config(self, cfg: Dict[str, Any]) -> None:
        """Reject config that silently collapses the cascade or divides by zero."""
        for key, default in (('path_tolerance_mm', 1.0),
                             ('max_line_length', 0.5),
                             ('max_arc_length', 0.5),
                             ('max_ptp_length', 0.1),
                             ('arc_gain', 1.5),
                             ('waypoint_spacing_mm', 10.0)):
            value = float(cfg.get(key, default))
            if not value > 0.0:
                raise ValueError(f'{key} must be > 0, got {value}')

        # Zero is a legitimate way to disable each of these, negative is not.
        for key, default in (('arc_strictness', 0.5),
                             ('min_arc_angle_deg', 15.0)):
            value = float(cfg.get(key, default))
            if value < 0.0:
                raise ValueError(f'{key} must be >= 0, got {value}')

        if self.min_fit_points < 3:
            raise ValueError(
                f'min_fit_points must be >= 3, got {self.min_fit_points}')

    def process_path(
        self,
        seam_points: List[SeamPoint],
        config: Optional[Dict[str, Any]] = None,
        is_closed: bool = False,
    ) -> List[Seam]:
        """Process ordered SeamPoints into classified Seam objects.

        Args:
            seam_points: Ordered SeamPoint list from the extractor.
            config:      Optional per-call config overriding construction config.
            is_closed:   True when the points form a closed loop; the loop is
                         closed by wrapping the first point so a full circle
                         can classify as one arc.

        Returns:
            List of Seam objects. Empty if fewer than 2 valid points.
        """
        if config is not None:
            self._apply_config(config)

        if len(seam_points) < 2:
            logger.warning(f'Too few SeamPoints to process: {len(seam_points)}')
            return []

        # The fits below are SVD-based: a single non-finite coordinate raises
        # LinAlgError from inside LAPACK, naming neither the chain nor the point.
        raw = [np.asarray(sp.position, dtype=float).ravel()
               for sp in seam_points]
        if any(p.shape != (3,) for p in raw):
            logger.warning('SeamPoint positions are not 3D; skipping chain')
            return []
        bad = int(sum(not np.isfinite(p).all() for p in raw))
        if bad:
            logger.warning(
                f'{bad} of {len(raw)} SeamPoint position(s) non-finite; '
                'skipping chain'
            )
            return []

        working = list(seam_points)
        if is_closed and len(working) >= 3:
            working.append(working[0])

        sublists = self._split_on_contact_type(working)

        seams: List[Seam] = []
        for sublist in sublists:
            positions = np.array([sp.position for sp in sublist])
            for seg_points, seg_type in self._classify(positions):
                for split_pts in self._split_by_length(seg_points, seg_type):
                    seam = self._wrap_in_seam(split_pts, seg_type, sublist)
                    if seam is not None:
                        seams.append(seam)

        logger.info(f'PathCreator produced {len(seams)} Seam object(s)')
        return seams

    def _split_on_contact_type(
        self, seam_points: List[SeamPoint]
    ) -> List[List[SeamPoint]]:
        """Group the chain into sublists of uniform joint character.

        The character is (is_edge_joint, owner_side); a change in either ends
        a sublist. Runs shorter than `min_contact_run` are flicker at ambiguous
        zones, not genuine changes, and are absorbed into their longer
        neighbour. The test is on LENGTH, not point count: one chain crosses
        both meshes, which sample at very different densities.
        """
        runs: List[List[Any]] = []  # [type, count]
        for sp in seam_points:
            t = (sp.on_edge_1 and sp.on_edge_2, sp.owner_side)
            if runs and runs[-1][0] == t:
                runs[-1][1] += 1
            else:
                runs.append([t, 1])

        positions = np.asarray([sp.position for sp in seam_points], dtype=float)
        cumulative = np.concatenate((
            [0.0], np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        ))

        while len(runs) > 1:
            # Arc length per run, recomputed because absorption mutates `runs`.
            lengths, start = [], 0
            for _, count in runs:
                end = start + count
                lengths.append(float(cumulative[end - 1] - cumulative[start]))
                start = end

            shortest = int(np.argmin(lengths))
            if lengths[shortest] >= self.min_contact_run:
                break
            neighbours = [i for i in (shortest - 1, shortest + 1)
                          if 0 <= i < len(runs)]
            absorber = max(neighbours, key=lambda i: lengths[i])
            runs[absorber][1] += runs[shortest][1]
            runs.pop(shortest)
            # Re-merge neighbours that now share a type.
            i = 1
            while i < len(runs):
                if runs[i][0] == runs[i - 1][0]:
                    runs[i - 1][1] += runs[i][1]
                    runs.pop(i)
                else:
                    i += 1

        sublists: List[List[SeamPoint]] = []
        cursor = 0
        for _, count in runs:
            chunk = seam_points[cursor: cursor + count]
            cursor += count
            if len(chunk) >= 2:
                sublists.append(chunk)
        return sublists

    def _classify(self, positions: NDArray) -> List[Tuple[NDArray, str]]:
        """Greedy tolerance-cascade consumer over the ordered positions."""
        n = len(positions)
        segments: List[Tuple[NDArray, str]] = []
        k = 0
        ptp_start: Optional[int] = None

        while k < n:
            remaining = n - k
            if remaining < self.min_fit_points:
                if ptp_start is None:
                    ptp_start = k
                k = n
                break

            m_line = self._grow_line(positions, k)
            m_arc = self._grow_arc(positions, k)

            take_type: Optional[str] = None
            take_end = k

            line_ok = (m_line - k) >= self.min_fit_points
            arc_ok = (m_arc - k) >= self.min_fit_points

            if arc_ok and (not line_ok or m_arc - k >= self.arc_gain * (m_line - k)):
                take_type, take_end = 'arc', m_arc
            elif line_ok:
                take_type, take_end = 'line', m_line

            if take_type is None:
                if ptp_start is None:
                    ptp_start = k
                k += 1
                continue

            if ptp_start is not None:
                segments.append((positions[ptp_start: k + 1], 'ptp'))
                ptp_start = None

            segments.append((positions[k:take_end], take_type))
            # Segments share their junction point for path continuity.
            k = take_end - 1 if take_end < n else n

        if ptp_start is not None and n - ptp_start >= 2:
            segments.append((positions[ptp_start:n], 'ptp'))

        return [(pts, t) for pts, t in segments if len(pts) >= 2]

    def _grow_line(self, positions: NDArray, k: int) -> int:
        """Largest m such that positions[k:m] fits a line within tolerance."""
        n = len(positions)
        m = k + 2
        while m < n:
            if self._line_max_deviation(positions[k: m + 1]) > self.tolerance:
                break
            m += 1
        return m

    def _grow_arc(self, positions: NDArray, k: int) -> int:
        """Largest m such that positions[k:m] fits a circle within the strict
        arc tolerance and subtends at least the minimum arc angle."""
        n = len(positions)
        best = k
        m = k + self.min_fit_points
        while m <= n:
            fit = self._fit_circle(positions[k:m])
            if fit is None or fit['max_deviation'] > self.arc_tolerance:
                break
            if fit['subtended'] >= self.min_arc_angle:
                best = m
            m += 1
        return best

    def _line_max_deviation(self, points: NDArray) -> float:
        """Max perpendicular distance of points from their best-fit line."""
        centroid = points.mean(axis=0)
        centered = points - centroid
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        direction = vt[0]
        projected = np.outer(centered @ direction, direction)
        return float(np.max(np.linalg.norm(centered - projected, axis=1)))

    def _fit_circle(self, points: NDArray) -> Optional[Dict[str, Any]]:
        """Kasa circle fit in the PCA plane, with full 3D max deviation.

        Each point's deviation combines the in-plane radial error and the
        out-of-plane height, so helical or warped runs cannot pass as planar
        arcs. None on degenerate geometry.
        """
        if len(points) < 3:
            return None

        centroid = points.mean(axis=0)
        centered = points - centroid
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        e1, e2, e3 = vt[0], vt[1], vt[2]

        x = centered @ e1
        y = centered @ e2
        h = centered @ e3  # out-of-plane height

        A = np.column_stack([x, y, np.ones(len(points))])
        params, _, _, _ = np.linalg.lstsq(A, x ** 2 + y ** 2, rcond=None)

        a, b = params[0] / 2.0, params[1] / 2.0
        r_sq = a ** 2 + b ** 2 + params[2]
        if r_sq <= 0.0:
            return None
        radius = float(np.sqrt(r_sq))

        radial = np.sqrt((x - a) ** 2 + (y - b) ** 2)
        deviation = np.sqrt((radial - radius) ** 2 + h ** 2)

        angles = np.arctan2(y - b, x - a)
        unwrapped = np.unwrap(angles)
        subtended = float(abs(unwrapped[-1] - unwrapped[0]))

        center = centroid + a * e1 + b * e2

        return {
            'center': center,
            'radius': radius,
            'max_deviation': float(np.max(deviation)),
            'subtended': subtended,
        }

    def _split_by_length(
        self, points: NDArray, seg_type: str
    ) -> List[NDArray]:
        """Split a segment into equal parts when it exceeds the type's max length."""
        max_len = {
            'line': self.max_line_length,
            'arc': self.max_arc_length,
        }.get(seg_type, self.max_ptp_length)

        step_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        total = float(np.sum(step_lengths))
        if total <= max_len:
            return [points]

        n_splits = int(np.ceil(total / max_len))
        target = total / n_splits

        result: List[NDArray] = []
        start = 0
        accumulated = 0.0
        for i in range(1, len(points)):
            accumulated += step_lengths[i - 1]
            if accumulated >= target and i < len(points) - 1:
                result.append(points[start: i + 1])
                start = i
                accumulated = 0.0

        tail = points[start:]
        if len(tail) >= 2:
            result.append(tail)
        elif result:
            result[-1] = np.vstack([result[-1], tail])

        return [r for r in result if len(r) >= 2]

    def _wrap_in_seam(
        self,
        points: NDArray,
        seg_type: str,
        seam_points_subset: List[SeamPoint],
    ) -> Optional[Seam]:
        """Wrap positions into a Seam with per-point normals and metadata."""
        if len(points) < 2:
            return None

        subset_positions = np.array([sp.position for sp in seam_points_subset])

        # SeamPoint names its normals for the surface they come off; the seam
        # config keys are the main/secondary vocabulary shared with the OCCT
        # extractor and WeldPlanner. base -> main, wall -> secondary.
        normals_main = []
        normals_secondary = []
        for pt in points:
            idx = int(np.argmin(np.linalg.norm(subset_positions - pt, axis=1)))
            normals_main.append(seam_points_subset[idx].normal_base)
            normals_secondary.append(seam_points_subset[idx].normal_wall)

        half = len(seam_points_subset) / 2.0
        on_edge_1 = sum(sp.on_edge_1 for sp in seam_points_subset) > half
        on_edge_2 = sum(sp.on_edge_2 for sp in seam_points_subset) > half

        try:
            if seg_type == 'line':
                seam = Seam(line_segment=LineSegment(start=points[0], end=points[-1]))
            elif seg_type == 'arc':
                fit = self._fit_circle(points)
                if fit is None:
                    seam = Seam(ptp_segment=PtPSegment(points=points))
                    seg_type = 'ptp'
                else:
                    seam = Seam(arc_segment=ArcSegment(
                        points=points, center=fit['center'], radius=fit['radius']
                    ))
            else:
                seam = Seam(ptp_segment=PtPSegment(points=points))
        except Exception as e:
            logger.warning(f'Segment construction failed in _wrap_in_seam: {e}')
            return None

        seam.config['is_edge_joint'] = on_edge_1 and on_edge_2
        seam.config['on_edge_1'] = on_edge_1
        seam.config['on_edge_2'] = on_edge_2
        seam.config['geometry_type'] = seg_type
        seam.config['smoothed_points'] = points
        seam.config['normals_main'] = np.array(normals_main)
        seam.config['normals_secondary'] = np.array(normals_secondary)

        return seam
