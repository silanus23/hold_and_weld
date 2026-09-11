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

from .params import PathCreatorParams
from .seam_point import SeamPoint
from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
from ..core.ptp_segment import PtPSegment
from ..core.seam import Seam

logger = logging.getLogger(__name__)


class PathCreator:
    """Classify ordered SeamPoints into segments wrapped in Seam objects."""

    def __init__(self, config: Optional[Dict[str, Any]] = None) -> None:
        """Initialize the segment classifier.

        Args:
            config: Optional config dict, per PathCreatorParams in
                params.py. Unknown keys are ignored, so the same dict can be
                handed to SeamExtractorMesh.

        Raises:
            ValueError: If a parameter is out of range.
        """
        self._init_config: Dict[str, Any] = config or {}
        self._apply_config(None)

    def _apply_config(self, config: Optional[Dict[str, Any]]) -> None:
        """Rebuild params from per-call config > init config > defaults."""
        self.cfg = PathCreatorParams.from_dict(
            {**self._init_config, **(config or {})})

    def process_path(
        self,
        seam_points: List[SeamPoint],
        config: Optional[Dict[str, Any]] = None,
        is_closed: bool = False,
    ) -> List[Seam]:
        """Process ordered SeamPoints into classified Seam objects.

        Args:
            config: Optional per-call config, overriding construction config.
            is_closed: True when the points form a closed loop; the loop is
                closed by wrapping the first point so a full circle can
                classify as one arc.

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
            for seg_points, seg_type, seg_start in self._classify(positions):
                for split_pts, offset in self._split_by_length(seg_points, seg_type):
                    seam = self._wrap_in_seam(
                        split_pts, seg_type, sublist, seg_start + offset)
                    if seam is not None:
                        seams.append(seam)

        self._join_consecutive(seams, is_closed)

        logger.info(f'PathCreator produced {len(seams)} Seam object(s)')
        return seams

    def _join_consecutive(self, seams: List[Seam], is_closed: bool) -> None:
        """Extend each seam to where the next one begins, in place.

        Splitting the chain into sublists loses the step across each
        boundary - `_classify` already shares a junction point between
        segments WITHIN a sublist; this extends the same rule across
        sublist boundaries, after fitting rather than before so a foreign
        sample never feeds a fit it doesn't belong to.
        """
        if len(seams) < 2:
            return

        pairs = list(zip(seams, seams[1:]))
        if is_closed:
            pairs.append((seams[-1], seams[0]))

        for seam, following in pairs:
            points = seam.config.get('smoothed_points')
            nxt = following.config.get('smoothed_points')
            if points is None or nxt is None or not len(nxt):
                continue
            if np.allclose(points[-1], nxt[0]):
                continue

            # A far larger gap than one ordinary sample means a segment went
            # missing (a `_wrap_in_seam` build failure) and the bridge crosses
            # unwelded metal. Still the best available path, so it's joined
            # and RECORDED rather than refused - the key rides to the JSON.
            gap = float(np.linalg.norm(nxt[0] - points[-1]))
            if len(points) > 1:
                spacing = float(np.median(
                    np.linalg.norm(np.diff(points, axis=0), axis=1)))
                if spacing > 1e-12 and gap > self.cfg.max_bridge_factor * spacing:
                    seam.config['bridged_gap_m'] = gap
                    logger.warning(
                        f'Bridging a {gap * 1000:.2f}mm gap to the next seam, '
                        f'past {self.cfg.max_bridge_factor}x the seam sample '
                        f'spacing of {spacing * 1000:.2f}mm; a segment is '
                        'missing between them and the bridge crosses unwelded '
                        'metal'
                    )

            seam.config['smoothed_points'] = np.vstack([points, nxt[0]])
            # The normals belong to the position, so take the NEXT seam's -
            # they were evaluated there. WeldPlanner requires one per point.
            for key in ('normals_main', 'normals_secondary'):
                have, take = seam.config.get(key), following.config.get(key)
                if have is None or take is None or not len(take):
                    continue
                seam.config[key] = np.vstack([have, take[0]])

            if seam.line_segment is not None:
                seam.line_segment.end = nxt[0]
            elif seam.arc_segment is not None:
                seam.arc_segment.points = seam.config['smoothed_points']
            elif seam.ptp_segment is not None:
                seam.ptp_segment.points = seam.config['smoothed_points']

    def _split_on_contact_type(
        self, seam_points: List[SeamPoint]
    ) -> List[List[SeamPoint]]:
        """Group the chain into sublists of uniform joint character.

        The character is (is_edge_joint, owner_side); a change in either
        ends a sublist. Runs shorter than `min_contact_run` are flicker at
        ambiguous zones and get absorbed into their longer same-mesh
        neighbour - by LENGTH, not point count, since the two meshes sample
        at very different densities. Never absorbed across a mesh handoff,
        however short: that would relabel the other mesh's points into a fit
        shaped by this mesh's geometry.
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

            # Shortest first, but a run whose neighbours are all on the other
            # mesh is skipped rather than merged, so the search has to carry on
            # to the next-shortest instead of stopping at the minimum.
            absorbed = False
            for shortest in np.argsort(lengths):
                shortest = int(shortest)
                if lengths[shortest] >= self.cfg.min_contact_run:
                    break
                side = runs[shortest][0][1]
                neighbours = [i for i in (shortest - 1, shortest + 1)
                              if 0 <= i < len(runs)
                              and runs[i][0][1] == side]
                if not neighbours:
                    continue
                absorber = max(neighbours, key=lambda i: lengths[i])
                runs[absorber][1] += runs[shortest][1]
                runs.pop(shortest)
                absorbed = True
                break
            if not absorbed:
                break

            # Re-merge neighbours that now share a type.
            i = 1
            while i < len(runs):
                if runs[i][0] == runs[i - 1][0]:
                    runs[i - 1][1] += runs[i][1]
                    runs.pop(i)
                else:
                    i += 1

        sublists: List[List[SeamPoint]] = []
        pending: List[SeamPoint] = []
        cursor = 0
        for _, count in runs:
            # Sublists are disjoint, so the step across each boundary is lost
            # here; _join_consecutive repairs it downstream after fitting.
            chunk = seam_points[cursor: cursor + count]
            cursor += count
            if len(chunk) >= 2:
                sublists.append(pending + chunk)
                pending = []
            elif sublists:
                # One point cannot carry a segment of its own; it is a
                # junction, so it goes to the sublist it follows rather than
                # being dropped and opening a gap in the path.
                sublists[-1].extend(chunk)
            else:
                # No sublist exists yet to follow, since this run is the
                # first in the chain; carried forward onto the first one
                # that does get created, rather than dropped here.
                pending.extend(chunk)

        if pending:
            if sublists:
                sublists[0][:0] = pending
            else:
                # The whole chain was single-point runs; too short for any
                # segment, but returned rather than dropped so the caller
                # sees it, not a silently empty chain.
                sublists.append(pending)
        return sublists

    def _classify(self, positions: NDArray) -> List[Tuple[NDArray, str, int]]:
        """Greedy tolerance-cascade consumer over the ordered positions.

        Returns:
            (points, type, start) per segment, where `start` indexes the
            segment's first point in `positions`. The index is carried rather
            than recovered later: the points are the only thing a segment
            keeps, and matching them back by position costs a scan per point
            and cannot separate two coincident samples.
        """
        n = len(positions)
        segments: List[Tuple[NDArray, str, int]] = []
        k = 0
        ptp_start: Optional[int] = None

        while k < n:
            remaining = n - k
            if remaining < self.cfg.min_fit_points:
                if ptp_start is None:
                    ptp_start = k
                k = n
                break

            m_line = self._grow_line(positions, k)
            m_arc = self._grow_arc(positions, k)

            take_type: Optional[str] = None
            take_end = k

            line_ok = (m_line - k) >= self.cfg.min_fit_points
            arc_ok = (m_arc - k) >= self.cfg.min_fit_points

            if arc_ok and (not line_ok or m_arc - k >= self.cfg.arc_gain * (m_line - k)):
                take_type, take_end = 'arc', m_arc
            elif line_ok:
                take_type, take_end = 'line', m_line

            if take_type is None:
                if ptp_start is None:
                    ptp_start = k
                k += 1
                continue

            if ptp_start is not None:
                segments.append((positions[ptp_start: k + 1], 'ptp', ptp_start))
                ptp_start = None

            segments.append((positions[k:take_end], take_type, k))
            # Segments share their junction point for path continuity.
            k = take_end - 1 if take_end < n else n

        if ptp_start is not None and n - ptp_start >= 2:
            segments.append((positions[ptp_start:n], 'ptp', ptp_start))

        return [(pts, t, start) for pts, t, start in segments if len(pts) >= 2]

    def _grow_line(self, positions: NDArray, k: int) -> int:
        """Largest m such that positions[k:m] fits a line within tolerance."""
        n = len(positions)
        m = k + 2
        while m < n:
            if self._line_max_deviation(positions[k: m + 1]) > self.cfg.tolerance:
                break
            m += 1
        return m

    def _grow_arc(self, positions: NDArray, k: int) -> int:
        """Largest m such that positions[k:m] fits an arc.

        The fit must hold the strict arc tolerance and subtend at least the
        minimum arc angle.
        """
        n = len(positions)
        best = k
        m = k + self.cfg.min_fit_points
        while m <= n:
            fit = self._fit_circle(positions[k:m])
            if fit is None or fit['max_deviation'] > self.cfg.arc_tolerance:
                break
            if fit['subtended'] >= self.cfg.min_arc_angle:
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
    ) -> List[Tuple[NDArray, int]]:
        """Split a segment into equal parts when it exceeds the type's max length.

        Returns:
            (points, offset) per part, where `offset` indexes the part's first
            point in `points`.
        """
        max_len = {
            'line': self.cfg.max_line_length,
            'arc': self.cfg.max_arc_length,
        }.get(seg_type, self.cfg.max_ptp_length)

        step_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        total = float(np.sum(step_lengths))
        if total <= max_len:
            return [(points, 0)]

        n_splits = int(np.ceil(total / max_len))
        target = total / n_splits

        result: List[Tuple[NDArray, int]] = []
        start = 0
        accumulated = 0.0
        for i in range(1, len(points)):
            accumulated += step_lengths[i - 1]
            if accumulated >= target and i < len(points) - 1:
                result.append((points[start: i + 1], start))
                start = i
                accumulated = 0.0

        tail = points[start:]
        if len(tail) >= 2:
            result.append((tail, start))
        elif result:
            # Absorbed into the part before it, which keeps that part's offset.
            last_points, last_start = result[-1]
            result[-1] = (np.vstack([last_points, tail]), last_start)

        return [(pts, offset) for pts, offset in result if len(pts) >= 2]

    def _wrap_in_seam(
        self,
        points: NDArray,
        seg_type: str,
        seam_points_subset: List[SeamPoint],
        start: int,
    ) -> Optional[Seam]:
        """Wrap positions into a Seam with per-point normals and metadata.

        Args:
            seam_points_subset: The SeamPoints the run was cut from.
            start: Index of `points[0]` within `seam_points_subset`.
        """
        if len(points) < 2:
            return None

        # base -> main, wall -> secondary: SeamPoint names normals for their
        # surface, Seam config uses the main/secondary vocabulary shared with
        # OCCT and WeldPlanner. Indexed directly rather than matched by
        # position - a closed loop revisits positions at its wrap, so a
        # nearest-position search would silently pick the wrong SeamPoint.
        owners = seam_points_subset[start: start + len(points)]
        if len(owners) != len(points):
            logger.warning(
                f'Segment of {len(points)} point(s) at index {start} runs past '
                f'its {len(seam_points_subset)}-point subset; skipping'
            )
            return None

        normals_main = [sp.normal_base for sp in owners]
        normals_secondary = [sp.normal_wall for sp in owners]

        half = len(seam_points_subset) / 2.0
        on_edge_1 = sum(sp.on_edge_1 for sp in seam_points_subset) > half
        on_edge_2 = sum(sp.on_edge_2 for sp in seam_points_subset) > half

        # A dropped segment is a HOLE, not a shorter weld: `_join_consecutive`
        # bridges straight across the gap, so the exported path runs through
        # unwelded metal. PtP has no fit to fail, so it's always the fallback
        # - only a PtP failure itself leaves nothing to return.
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
        except (ValueError, KeyError, np.linalg.LinAlgError) as e:
            logger.warning(
                f'{seg_type.upper()} construction failed over {len(points)} '
                f'point(s) ({e}); falling back to PtP over the same run'
            )
            try:
                seam = Seam(ptp_segment=PtPSegment(points=points))
            except (ValueError, np.linalg.LinAlgError) as ptp_error:
                logger.error(
                    f'PtP fallback also failed ({ptp_error}); the run from '
                    f'{points[0]} to {points[-1]} is DROPPED and the exported '
                    'path will bridge straight across it'
                )
                return None
            seg_type = 'ptp'

        seam.config['is_edge_joint'] = on_edge_1 and on_edge_2
        seam.config['on_edge_1'] = on_edge_1
        seam.config['on_edge_2'] = on_edge_2
        seam.config['geometry_type'] = seg_type
        seam.config['smoothed_points'] = points
        seam.config['normals_main'] = np.array(normals_main)
        seam.config['normals_secondary'] = np.array(normals_secondary)

        return seam
