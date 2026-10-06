# Copyright 2026 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""PathCreator - Classify ordered SeamPoints into geometric segments.

A run is a LINE when a straight line holds the process path tolerance, an ARC when a circle holds
the stricter arc tolerance over a minimum angle, and PTP otherwise. Line wins ties; forcing a
near-circle into an arc leaves the seam, while demoting a true arc to PTP only densifies waypoints.
"""

from dataclasses import replace
import logging
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .params import PathCreatorParams
from .seam_point import SeamPoint
from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
from ..core.ptp_segment import PtPSegment
from ..core.seam import Seam, SeamConfig

logger = logging.getLogger(__name__)


class PathCreator:
    """Classify ordered SeamPoints into segments wrapped in Seam objects."""

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        """Initialize the segment classifier."""
        self.cfg = PathCreatorParams.from_dict(config)

    def process_path(self, seam_points: list[SeamPoint], is_closed: bool = False) -> list[Seam]:
        """Process ordered SeamPoints into classified Seam objects.

        Args:
            is_closed: True when the points form a closed loop; the loop is
                closed by wrapping the first point so a full circle can
                classify as one arc.

        Returns:
            List of Seam objects.

        Raises:
            ValueError: If the chain has fewer than 2 points, a normal is not 3D, or a position
                is not a finite 3D point. Refused rather than skipped: a job missing a seam is the
                wrong job.
        """
        if len(seam_points) < 2:
            raise ValueError(f'Too few SeamPoints to process: {len(seam_points)}')

        if any(np.shape(vector) != (3,) for sp in seam_points
               for vector in (sp.position, sp.normal_base, sp.normal_wall)):
            raise ValueError('SeamPoint positions and normals must be 3D')
        bad = sum(not np.isfinite(sp.position).all() for sp in seam_points)
        if bad:
            raise ValueError(
                f'{bad} of {len(seam_points)} SeamPoint position(s) are non-finite')

        working = list(seam_points)
        if is_closed and len(working) >= 3:
            working.append(working[0])

        sublists = self._split_on_contact_type(working)

        seams: list[Seam] = []
        for sublist in sublists:
            positions = np.array([sp.position for sp in sublist])
            for seg_points, seg_type, seg_start in self._classify(positions):
                for split_pts, offset in self._split_by_length(seg_points, seg_type):
                    seams.append(self._wrap_in_seam(
                        split_pts, seg_type, sublist, seg_start + offset))

        self._join_consecutive(seams, is_closed)

        kinds = [seam.segment_type for seam in seams]
        line, arc, ptp = (kinds.count(kind) for kind in ('line', 'arc', 'ptp'))
        logger.info(f'PathCreator produced {len(seams)} seams: {line} line, {arc} arc, {ptp} PtP')
        return seams

    def _join_consecutive(self, seams: list[Seam], is_closed: bool) -> None:
        """Extend each seam to where the next one begins, in place.

        Splitting the chain into sublists loses the step across each boundary - `_classify` already
        shares a junction point between segments WITHIN a sublist; this extends the same rule
        across sublist boundaries, after fitting rather than before so a foreign sample never feeds
        a fit it doesn't belong to.
        """
        if len(seams) < 2:
            return

        pairs = list(zip(seams, seams[1:]))
        if is_closed:
            pairs.append((seams[-1], seams[0]))

        for seam, following in pairs:
            config, nxt = seam.config, following.config
            if np.array_equal(config.smoothed_points[-1], nxt.smoothed_points[0]):
                continue

            # The position is the next seam's start, but the pose there is still welded against
            # THIS seam's faces, so the normals repeat its own last ones; the next seam's can
            # belong to the other mesh and flip the torch. WeldPlanner requires one per point.
            seam.config = replace(
                config,
                smoothed_points=np.vstack([config.smoothed_points, nxt.smoothed_points[0]]),
                normals_main=np.vstack([config.normals_main, config.normals_main[-1]]),
                normals_secondary=np.vstack(
                    [config.normals_secondary, config.normals_secondary[-1]]),
            )

            if seam.line_segment is not None:
                seam.line_segment.end = nxt.smoothed_points[0]
            elif seam.arc_segment is not None:
                seam.arc_segment.points = seam.config.smoothed_points
            elif seam.ptp_segment is not None:
                seam.ptp_segment.points = seam.config.smoothed_points

    def _split_on_contact_type(self, seam_points: list[SeamPoint]) -> list[list[SeamPoint]]:
        """Group the chain into sublists of uniform joint character.

        The character is (is_edge_joint, owner_side); a change in either ends a sublist, except
        that an edge joint ignores owner_side: both meshes end on the seam there, so the owner is
        a tie-break that flickers. Short runs (by LENGTH, since the two meshes sample at very
        different densities) are flicker at ambiguous zones and get absorbed into a same-side
        neighbour, never across a mesh handoff. A single-point run can't be fitted alone, so it
        rides with an adjacent sublist instead; `_wrap_in_seam` gives it that sublist's normals.
        """
        runs: list[list[Any]] = []
        for sp in seam_points:
            edge_joint = sp.on_edge_1 and sp.on_edge_2
            t = (True, 0) if edge_joint else (False, sp.owner_side)
            if runs and runs[-1][0] == t:
                runs[-1][1] += 1
            else:
                runs.append([t, 1])

        positions = np.asarray([sp.position for sp in seam_points], dtype=float)
        step_dists = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        cumulative = np.insert(np.cumsum(step_dists), 0, 0.0)

        while len(runs) > 1:
            # Arc length per run, recomputed because absorption mutates `runs`.
            lengths, start = [], 0
            for _, count in runs:
                end = start + count
                lengths.append(float(cumulative[end - 1] - cumulative[start]))
                start = end

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

            # Absorbing a run can leave two runs of the same character side by side; fuse them.
            i = 1
            while i < len(runs):
                if runs[i][0] == runs[i - 1][0]:
                    runs[i - 1][1] += runs[i][1]
                    runs.pop(i)
                else:
                    i += 1

        sublists: list[list[SeamPoint]] = []
        pending: list[SeamPoint] = []
        cursor = 0
        for _, count in runs:
            chunk = seam_points[cursor: cursor + count]
            cursor += count
            if len(chunk) >= 2:
                sublists.append(pending + chunk)
                pending = []
            elif sublists:
                sublists[-1].extend(chunk)
            else:
                pending.extend(chunk)

        if pending:
            if sublists:
                sublists[0][:0] = pending
            else:
                sublists.append(pending)
        return sublists

    def _classify(self, positions: NDArray) -> list[tuple[NDArray, str, int]]:
        """Greedy tolerance-cascade consumer over the ordered positions.

        Returns:
            (points, type, start) per segment, where `start` indexes the segment's first point in
            `positions`. The index is carried rather than recovered later: the points are the only
            thing a segment keeps, and matching them back by position costs a scan per point and
            cannot separate two coincident samples.
        """
        n = len(positions)
        segments: list[tuple[NDArray, str, int]] = []
        k = 0
        ptp_start: int | None = None

        while k < n:
            remaining = n - k
            if remaining < self.cfg.min_fit_points:
                if ptp_start is None:
                    ptp_start = k
                break

            m_line = self._grow_line(positions, k)
            m_arc = self._grow_arc(positions, k)

            take_type: str | None = None
            take_end = k

            len_line = m_line - k
            len_arc = m_arc - k
            line_ok = len_line >= self.cfg.min_fit_points
            arc_ok = len_arc >= self.cfg.min_fit_points

            if arc_ok and (not line_ok or len_arc >= self.cfg.arc_gain * len_line):
                take_type, take_end = 'arc', m_arc
            elif line_ok:
                take_type, take_end = 'line', m_line

            if take_type is None:
                if ptp_start is None:
                    ptp_start = k
                k += 1
                continue

            if ptp_start is not None:
                # Through k: the PTP run ends on the next segment's first point, the shared
                # junction.
                segments.append((positions[ptp_start: k + 1], 'ptp', ptp_start))
                ptp_start = None

            segments.append((positions[k:take_end], take_type, k))
            # Segments share their junction point for path continuity.
            k = take_end - 1 if take_end < n else n

        if ptp_start is not None:
            segments.append((positions[ptp_start:n], 'ptp', ptp_start))

        return [(pts, t, start) for pts, t, start in segments if len(pts) >= 2]

    def _grow_line(self, positions: NDArray, k: int) -> int:
        """Largest m such that positions[k:m] fits a line within tolerance."""
        n = len(positions)
        m = k + 2
        while m < n:
            if self._line_max_deviation(positions[k: m + 1]) > self.cfg.path_tolerance:
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
        """Max perpendicular distance of points from the chord joining the first and last.

        The chord, not a best-fit line: the welder runs one LIN from the first point to the
        last, and a one-sided bow sits up to twice as far from that chord as from its fit.
        A run whose ends coincide has no chord and is never a line.
        """
        chord = points[-1] - points[0]
        length = np.linalg.norm(chord)
        if length < 1e-10:
            return float('inf')
        direction = chord / length
        offsets = points - points[0]
        projected = np.outer(offsets @ direction, direction)
        return float(np.max(np.linalg.norm(offsets - projected, axis=1)))

    def _fit_circle(self, points: NDArray) -> dict[str, Any] | None:
        """Kasa circle fit in the PCA plane, with full 3D max deviation.

        Each point's deviation combines the in-plane radial error and the out-of-plane height, so
        helical or warped runs cannot pass as planar arcs. None on degenerate geometry.
        """
        if len(points) < 3:
            return None

        centroid = points.mean(axis=0)
        centered = points - centroid
        # Rows of vt by falling spread: e1, e2 span the best-fit plane, e3 is its normal.
        _, _, vt = np.linalg.svd(centered, full_matrices=False)
        e1, e2, e3 = vt[0], vt[1], vt[2]

        x = centered @ e1
        y = centered @ e2
        h = centered @ e3

        # Kasa fit, linear in (2a, 2b, c):
        #   (x - a)^2 + (y - b)^2 = r^2  <=>  x^2 + y^2 = 2a*x + 2b*y + c,  c = r^2 - a^2 - b^2
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

    def _split_by_length(self, points: NDArray, seg_type: str) -> list[tuple[NDArray, int]]:
        """Split a segment into equal parts when it exceeds the type's max length.

        A closed arc is split in two whatever its length: the welder runs one Pilz CIRC per arc,
        from its first point to its last, and a full circle puts that goal on top of its start.
        """
        max_len = {
            'line': self.cfg.max_line_length,
            'arc': self.cfg.max_arc_length,
        }.get(seg_type, self.cfg.max_ptp_length)

        step_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
        total = float(np.sum(step_lengths))
        n_splits = int(np.ceil(total / max_len))
        if seg_type == 'arc' and np.linalg.norm(points[-1] - points[0]) <= self.cfg.path_tolerance:
            n_splits = max(n_splits, 2)
        if n_splits <= 1:
            return [(points, 0)]

        target = total / n_splits

        result: list[tuple[NDArray, int]] = []
        start = 0
        accumulated = 0.0
        for i in range(1, len(points)):
            accumulated += step_lengths[i - 1]
            if accumulated >= target and i < len(points) - 1:
                result.append((points[start: i + 1], start))
                start = i
                accumulated = 0.0

        result.append((points[start:], start))
        return result

    def _wrap_in_seam(
        self,
        points: NDArray,
        seg_type: str,
        seam_points_subset: list[SeamPoint],
        start: int,
    ) -> Seam:
        """Wrap positions into a Seam with per-point normals and metadata.

        `seam_points_subset` is the SeamPoints the run was cut from, and `start` the index of
        `points[0]` within it.
        """
        owners = seam_points_subset[start: start + len(points)]
        normals_main = np.array([sp.normal_base for sp in owners])
        normals_secondary = np.array([sp.normal_wall for sp in owners])

        # A point owned by the other mesh - edge-joint flicker, or the single point closing a
        # loop - carries that mesh's normals, which can point the opposite way. It takes those of
        # the nearest point owned by the seam's majority. A tie names no majority.
        side = np.array([sp.owner_side for sp in owners])
        count_1, count_2 = int((side == 1).sum()), int((side == 2).sum())
        if count_1 != count_2:
            majority = 1 if count_1 > count_2 else 2
            good = np.nonzero(side == majority)[0]
            bad = np.nonzero(side != majority)[0]
            if len(bad):
                nearest = good[np.argmin(np.abs(bad[:, None] - good[None, :]), axis=1)]
                normals_main[bad] = normals_main[nearest]
                normals_secondary[bad] = normals_secondary[nearest]

        half = len(seam_points_subset) / 2.0
        on_edge_1 = sum(sp.on_edge_1 for sp in seam_points_subset) > half
        on_edge_2 = sum(sp.on_edge_2 for sp in seam_points_subset) > half

        config = SeamConfig(
            smoothed_points=points,
            normals_main=normals_main,
            normals_secondary=normals_secondary,
            on_edge_1=on_edge_1,
            on_edge_2=on_edge_2,
        )

        # A piece `_split_by_length` cut from an arc can be too short to refit; it goes out as PTP.
        fit = self._fit_circle(points) if seg_type == 'arc' else None
        if seg_type == 'line':
            return Seam(line_segment=LineSegment(start=points[0], end=points[-1]), config=config)
        if fit is not None:
            return Seam(arc_segment=ArcSegment(
                points=points, center=fit['center'], radius=fit['radius']), config=config)
        return Seam(ptp_segment=PtPSegment(points=points), config=config)
