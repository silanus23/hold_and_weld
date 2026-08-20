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

"""Field-based weld seam extraction from a pair of touching mesh shells.

The seam is treated as a ridge of the local edge-response field rather than
as an ordered chain from the start:

    Pass 1 (situation):  unordered contact seeds from C++ get_contact_points
                         are probed against both meshes; each seed learns
                         which side carries the geometric edge (comparative
                         dihedral) or is discarded when neither does.
    Pass 2 (converge):   each surviving seed walks across the seam on its
                         edge side — perpendicular to the probed edge
                         direction, re-projected onto the surface — to the
                         maximum of the edge response, landing on the ridge.
    Pass 3 (order):      converged ridge points are coherence-filtered,
                         thinned, and chained into ordered seams using the
                         per-point edge direction; closed loops are detected
                         when chain ends meet.

Ordered per-seam SeamPoints are then delegated to PathCreator.
"""

from dataclasses import dataclass
import logging
from typing import Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from .edge_probe import EdgeProbe, ProbeMesh, ProbeResult
from ..core.seam import Seam

try:
    from . import mesh_intersection as _mesh_intersection
except ImportError as e:
    raise ImportError(
        'mesh_intersection C++ module not found. '
        'Build it with: cd mesh_intersection && mkdir build && cd build && cmake .. && make'
    ) from e

logger = logging.getLogger(__name__)

# All spatial scales below are multiples of the mean of the two meshes'
# median edge lengths — derived, not user parameters.
_WALK_BOUND_PROBE_RADII = 2.0   # walk half-span, in probe radii of the edge side
_THIN_RADIUS_EDGES = 0.25       # ridge thinning radius (merges duplicates only)
_LINK_RADIUS_EDGES = 3.0        # max neighbour distance when chaining
_COHERENCE_RADIUS_EDGES = 3.0   # neighbourhood for the local-line outlier test
_COHERENCE_MAX_RESIDUAL_EDGES = 0.5   # hard cap on distance from local ridge line
_COHERENCE_MIN_RESIDUAL_EDGES = 0.25  # floor, so grid-exact regions cannot
#                                       set a threshold that rejects normal
#                                       convergence scatter elsewhere
_MIN_CLOSED_CHAIN = 6           # minimum points for closed-loop detection
_MIN_LINK_ALIGNMENT = 0.5       # |cos| between link and local edge direction
_CORNER_RADIUS_PROBE_RADII = 2.5  # max end-to-end gap bridged at a corner
_CORNER_TANGENT_POINTS = 5      # chain-end points used for the end tangent


@dataclass
class SeamPoint:
    """Single ordered point on a weld seam with surface normal information.

    Attributes:
        position:          Converged 3D position on the geometric edge (3,).
        normal_main:       Surface normal of the base (non-edge) side (3,).
        normal_secondary:  Wall normal of the edge side (3,).
        on_edge_1:         True if mesh_1 shows a geometric edge here.
        on_edge_2:         True if mesh_2 shows a geometric edge here.
        refined_side:      Mesh the point converged on: 1 or 2.
    """

    position: NDArray
    normal_main: NDArray
    normal_secondary: NDArray
    on_edge_1: bool
    on_edge_2: bool
    refined_side: int


@dataclass
class _RidgePoint:
    """Internal: one converged ridge point with its cached edge-side probe."""

    position: NDArray
    side: int
    result: ProbeResult


class SeamExtractor:
    """Extract weld seams from two touching mesh shells (field pipeline)."""

    def __init__(
        self,
        mesh_1: trimesh.Trimesh,
        mesh_2: trimesh.Trimesh,
        params: Dict,
    ) -> None:
        """
        Args:
            mesh_1: First mesh (world frame, must be watertight).
            mesh_2: Second mesh (world frame, must be watertight).
            params: Configuration dict; reads 'epsilon' plus the EdgeProbe
                    keys 'edge_angle_min_deg' and 'gap_dominance'.

        Raises:
            ValueError: If either mesh is not watertight.
        """
        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')

        self.mesh_1 = mesh_1
        self.mesh_2 = mesh_2
        self.params = params
        self.epsilon = params.get('epsilon', 1e-3)

        self.probe = EdgeProbe(params)
        self.pm = {1: ProbeMesh(mesh_1), 2: ProbeMesh(mesh_2)}
        self.mean_edge = 0.5 * (self.pm[1].median_edge + self.pm[2].median_edge)

        # Median perpendicular residual of ridge points against their local
        # line fit; measured in the coherence pass, consumed by PathCreator
        # as a noise floor for fit tolerances.
        self.ridge_jitter: float = 0.0

    # ------------------------------------------------------------------ seeds

    def _collect_seeds(self) -> NDArray:
        """Pool unordered contact-boundary seeds from both call directions."""
        arrays = []
        for ma, mb in ((self.mesh_1, self.mesh_2), (self.mesh_2, self.mesh_1)):
            result = _mesh_intersection.get_contact_points(
                np.asarray(ma.vertices, dtype=np.float64),
                np.asarray(ma.faces, dtype=np.int32),
                np.asarray(mb.vertices, dtype=np.float64),
                np.asarray(mb.faces, dtype=np.int32),
                self.epsilon,
            )
            points = np.asarray(result['points'], dtype=np.float64)
            if len(points):
                arrays.append(points)

        if not arrays:
            return np.empty((0, 3))

        return np.vstack(arrays)

    # ---------------------------------------------------------------- passes

    def _classify_seed(
        self, position: NDArray
    ) -> List[Tuple[int, ProbeResult]]:
        """Pass 1: edge-carrying side(s) for one seed, best first.

        Returns up to two (side, probe_result) entries ordered by dihedral,
        so pass 2 can fall back to the other side when the first walk lands
        away from the contact zone (e.g. on an unrelated part edge).
        """
        r1 = self.probe.probe(self.pm[1], position)
        r2 = self.probe.probe(self.pm[2], position)

        candidates = []
        if r1.is_edge:
            candidates.append((1, r1))
        if r2.is_edge:
            candidates.append((2, r2))
        candidates.sort(key=lambda c: -c[1].dihedral)
        return candidates

    def _near_contact(self, side: int, position: NDArray) -> bool:
        """True if position lies close enough to the OTHER mesh to be a weld.

        A weld seam exists where the parts meet: a ridge point converged on
        one mesh must also be near the other. This rejects genuine but
        irrelevant part edges (e.g. the base plate's outer boundary) that the
        probe can latch onto.
        """
        other = self.pm[2 if side == 1 else 1]
        distance, _ = other.centroid_tree.query(position)
        return bool(distance <= self.epsilon + 2.0 * self.mean_edge)

    def _project_to_surface(self, pm: ProbeMesh, point: NDArray) -> NDArray:
        """Project point onto the mesh, restricted to the local face patch."""
        faces = self.probe.collect_faces(pm, point)
        if not faces:
            return point

        triangles = pm.mesh.triangles[faces]
        candidates = trimesh.triangles.closest_point(
            triangles, np.tile(point, (len(faces), 1))
        )
        distances = np.linalg.norm(candidates - point, axis=1)
        return candidates[int(np.argmin(distances))]

    def _walk_to_ridge(
        self,
        side: int,
        position: NDArray,
        seed_result: ProbeResult,
    ) -> Optional[_RidgePoint]:
        """Pass 2: walk across the seam to the edge-response maximum."""
        pm = self.pm[side]

        direction = np.cross(seed_result.edge_dir, seed_result.n_dominant)
        norm = np.linalg.norm(direction)
        if norm < 1e-10:
            return None
        direction /= norm

        step = pm.median_edge
        bound = _WALK_BOUND_PROBE_RADII * pm.probe_radius
        offsets = np.arange(-bound, bound + 0.5 * step, step)

        stations: List[Tuple[float, NDArray, ProbeResult]] = []
        for s in offsets:
            candidate = self._project_to_surface(pm, position + s * direction)
            result = self.probe.probe(pm, candidate)
            stations.append((s, candidate, result))

        best_idx = int(np.argmax([st[2].response for st in stations]))
        _, best_pos, best_result = stations[best_idx]

        # One parabolic refinement between the best station's neighbours.
        if 0 < best_idx < len(stations) - 1:
            s_prev, _, r_prev = stations[best_idx - 1]
            s_best, _, r_best = stations[best_idx]
            s_next, _, r_next = stations[best_idx + 1]
            denominator = r_prev.response - 2.0 * r_best.response + r_next.response
            if abs(denominator) > 1e-12:
                s_ref = s_best + 0.5 * step * (
                    (r_prev.response - r_next.response) / denominator
                )
                if s_prev < s_ref < s_next:
                    candidate = self._project_to_surface(
                        pm, position + s_ref * direction
                    )
                    result = self.probe.probe(pm, candidate)
                    if result.response > best_result.response:
                        best_pos, best_result = candidate, result

        if not best_result.is_edge:
            return None

        return _RidgePoint(position=best_pos, side=side, result=best_result)

    def _coherence_filter(
        self, ridge: List[_RidgePoint]
    ) -> List[_RidgePoint]:
        """Reject converged points that disagree with the local ridge line.

        Fits a line (position covariance) through each point's ridge
        neighbourhood and rejects points whose perpendicular residual is an
        outlier. Also measures the ridge jitter used downstream as a noise
        floor. Isolated points (fewer than 2 neighbours) are dropped.
        """
        if len(ridge) < 3:
            return ridge

        positions = np.array([rp.position for rp in ridge])
        tree = KDTree(positions)
        radius = _COHERENCE_RADIUS_EDGES * self.mean_edge

        residuals = np.full(len(ridge), np.inf)
        for i, rp in enumerate(ridge):
            idxs = tree.query_ball_point(rp.position, radius)
            if len(idxs) < 3:
                continue  # isolated: residual stays inf, point is dropped
            local = positions[idxs]
            centroid = local.mean(axis=0)
            centered = local - centroid
            _, _, vt = np.linalg.svd(centered, full_matrices=False)
            axis = vt[0]
            offset = rp.position - centroid
            residuals[i] = float(
                np.linalg.norm(offset - np.dot(offset, axis) * axis)
            )

        finite = residuals[np.isfinite(residuals)]
        if len(finite) == 0:
            return []

        self.ridge_jitter = float(np.median(finite))
        threshold = min(
            max(
                4.0 * self.ridge_jitter,
                _COHERENCE_MIN_RESIDUAL_EDGES * self.mean_edge,
            ),
            _COHERENCE_MAX_RESIDUAL_EDGES * self.mean_edge,
        )

        kept = [rp for rp, r in zip(ridge, residuals) if r <= threshold]
        logger.debug(
            f'Coherence filter kept {len(kept)}/{len(ridge)} point(s), '
            f'jitter={self.ridge_jitter * 1000:.3f}mm'
        )
        return kept

    def _thin(self, ridge: List[_RidgePoint]) -> List[_RidgePoint]:
        """Keep the highest-response representative per thinning radius."""
        if not ridge:
            return ridge

        order = np.argsort([-rp.result.response for rp in ridge])
        radius = _THIN_RADIUS_EDGES * self.mean_edge

        kept: List[_RidgePoint] = []
        kept_positions: List[NDArray] = []
        for i in order:
            rp = ridge[int(i)]
            if kept_positions:
                d = np.linalg.norm(
                    np.array(kept_positions) - rp.position, axis=1
                )
                if np.min(d) < radius:
                    continue
            kept.append(rp)
            kept_positions.append(rp.position)

        return kept

    def _order_ridge(
        self, ridge: List[_RidgePoint]
    ) -> List[Tuple[List[_RidgePoint], bool]]:
        """Pass 3: chain thinned ridge points into ordered seams.

        Greedy two-ended growth: from each chain end, link to the nearest
        unused point whose direction from the end aligns with the local edge
        direction and does not backtrack. Ends meeting closes the loop.
        """
        if len(ridge) < 2:
            return []

        positions = np.array([rp.position for rp in ridge])
        tree = KDTree(positions)
        link_radius = _LINK_RADIUS_EDGES * self.mean_edge

        unused = set(range(len(ridge)))
        chains: List[Tuple[List[_RidgePoint], bool]] = []

        seed_order = np.argsort([-rp.result.response for rp in ridge])

        def next_link(current: int, travel: Optional[NDArray]) -> Optional[int]:
            candidates = tree.query_ball_point(positions[current], link_radius)
            best_j, best_d = None, np.inf
            for j in candidates:
                if j not in unused:
                    continue
                delta = positions[j] - positions[current]
                d = np.linalg.norm(delta)
                if d < 1e-12:
                    continue
                u = delta / d
                edge_dir = ridge[current].result.edge_dir
                if edge_dir is not None and \
                        abs(np.dot(u, edge_dir)) < _MIN_LINK_ALIGNMENT:
                    continue
                if travel is not None and np.dot(u, travel) < 0.0:
                    continue
                if d < best_d:
                    best_j, best_d = j, d
            return best_j

        for seed in seed_order:
            seed = int(seed)
            if seed not in unused:
                continue
            unused.discard(seed)
            chain = [seed]

            # Grow forward from the tail, then backward from the head.
            for backward in (False, True):
                while True:
                    current = chain[0] if backward else chain[-1]
                    if len(chain) >= 2:
                        prev = chain[1] if backward else chain[-2]
                        travel = positions[current] - positions[prev]
                        n = np.linalg.norm(travel)
                        travel = travel / n if n > 1e-12 else None
                    else:
                        travel = None
                    j = next_link(current, travel)
                    if j is None:
                        break
                    unused.discard(j)
                    if backward:
                        chain.insert(0, j)
                    else:
                        chain.append(j)

            if len(chain) < 2:
                continue

            is_closed = (
                len(chain) >= _MIN_CLOSED_CHAIN
                and np.linalg.norm(positions[chain[0]] - positions[chain[-1]])
                <= link_radius
            )
            chains.append(([ridge[i] for i in chain], is_closed))

        return chains

    def _end_tangent(
        self, chain: List[_RidgePoint], at_head: bool
    ) -> Tuple[NDArray, NDArray]:
        """Return (end_position, outward unit tangent) of one chain end."""
        pts = [
            rp.position
            for rp in (chain[:_CORNER_TANGENT_POINTS] if at_head
                       else chain[-_CORNER_TANGENT_POINTS:])
        ]
        local = np.array(pts)
        centroid = local.mean(axis=0)
        _, _, vt = np.linalg.svd(local - centroid, full_matrices=False)
        tangent = vt[0]
        end_pos = local[0] if at_head else local[-1]
        if np.dot(end_pos - centroid, tangent) < 0.0:
            tangent = -tangent
        return end_pos, tangent

    def _merge_continuations(
        self, chains: List[Tuple[List[_RidgePoint], bool]]
    ) -> List[Tuple[List[_RidgePoint], bool]]:
        """Splice chains whose ends continue each other (pass 3.25).

        Chaining can break at ambiguity holes (e.g. where two seams cross);
        the fragments are collinear/co-curved continuations. Two open-chain
        ends are merged when they are within the link radius and both end
        tangents align with the joining direction. Repeats until stable,
        then re-checks loop closure.
        """
        link_radius = _LINK_RADIUS_EDGES * self.mean_edge
        merged = True
        while merged:
            merged = False
            for i in range(len(chains)):
                if merged:
                    break
                for j in range(i + 1, len(chains)):
                    chain_i, closed_i = chains[i]
                    chain_j, closed_j = chains[j]
                    if closed_i or closed_j:
                        continue
                    if len(chain_i) < 3 or len(chain_j) < 3:
                        continue

                    best = None
                    for hi in (True, False):
                        for hj in (True, False):
                            p_i, t_i = self._end_tangent(chain_i, hi)
                            p_j, t_j = self._end_tangent(chain_j, hj)
                            gap = float(np.linalg.norm(p_j - p_i))
                            if gap > link_radius or gap < 1e-12:
                                continue
                            join = (p_j - p_i) / gap
                            # Outward tangents must both align with the join.
                            if (np.dot(t_i, join) < 0.8
                                    or np.dot(t_j, -join) < 0.8):
                                continue
                            if best is None or gap < best[0]:
                                best = (gap, hi, hj)

                    if best is None:
                        continue

                    _, hi, hj = best
                    part_i = list(reversed(chain_i)) if hi else list(chain_i)
                    part_j = list(chain_j) if hj else list(reversed(chain_j))
                    chains[i] = (part_i + part_j, False)
                    chains.pop(j)
                    merged = True
                    break

        # Re-check loop closure on the merged chains.
        rechecked = []
        for chain, is_closed in chains:
            if not is_closed and len(chain) >= _MIN_CLOSED_CHAIN:
                gap = np.linalg.norm(chain[0].position - chain[-1].position)
                if gap <= link_radius:
                    is_closed = True
            rechecked.append((chain, is_closed))
        return rechecked

    def _extend_corners(
        self, chains: List[Tuple[List[_RidgePoint], bool]]
    ) -> None:
        """Reconstruct corner points the probe cannot see (pass 3.5, in place).

        Ridge coverage stops roughly one probe radius short of a corner,
        where the probe ball straddles two edge directions and the gap test
        degrades. For each pair of nearby open-chain ends, extend the two
        end tangent lines to their closest mutual point and append it to
        both chains, restoring the full seam length.
        """
        ends = []  # (chain_idx, at_head, position, outward_tangent, side)
        for ci, (chain, is_closed) in enumerate(chains):
            if is_closed or len(chain) < 3:
                continue
            for at_head in (True, False):
                end_pos, tangent = self._end_tangent(chain, at_head)
                ends.append((ci, at_head, end_pos, tangent, chain[0].side))

        pairs = []
        for i in range(len(ends)):
            for j in range(i + 1, len(ends)):
                if ends[i][0] == ends[j][0]:
                    continue  # never bridge a chain to itself
                gap = float(np.linalg.norm(ends[i][2] - ends[j][2]))
                radius = _CORNER_RADIUS_PROBE_RADII * max(
                    self.pm[ends[i][4]].probe_radius,
                    self.pm[ends[j][4]].probe_radius,
                )
                if gap <= radius:
                    pairs.append((gap, i, j, radius))
        pairs.sort()

        used = set()
        for gap, i, j, radius in pairs:
            if i in used or j in used:
                continue
            _, _, p1, t1, _ = ends[i]
            _, _, p2, t2, _ = ends[j]

            # Closest point between the two end lines p = p1 + s*t1, q = p2 + u*t2.
            cross = np.cross(t1, t2)
            denom = float(np.dot(cross, cross))
            if denom < 1e-10:
                continue  # near-parallel ends: not a corner
            w = p2 - p1
            s = float(np.dot(np.cross(w, t2), cross)) / denom
            u = float(np.dot(np.cross(w, t1), cross)) / denom
            if s <= 0.0 or u <= 0.0:
                continue  # intersection behind an end: not a corner
            corner = 0.5 * ((p1 + s * t1) + (p2 + u * t2))
            if (np.linalg.norm(corner - p1) > radius
                    or np.linalg.norm(corner - p2) > radius):
                continue

            used.update((i, j))
            for end in (ends[i], ends[j]):
                ci, at_head = end[0], end[1]
                chain = chains[ci][0]
                template = chain[0] if at_head else chain[-1]
                corner_point = _RidgePoint(
                    position=corner, side=template.side, result=template.result
                )
                if at_head:
                    chain.insert(0, corner_point)
                else:
                    chain.append(corner_point)

    # ------------------------------------------------------------- assembly

    def _build_seam_point(self, rp: _RidgePoint) -> SeamPoint:
        """Assemble a SeamPoint: probe the opposite side at the ridge position.

        normal_main is the base-surface normal from the non-edge side;
        normal_secondary is the wall normal — whichever of the edge side's
        two cluster normals is most orthogonal to normal_main.
        """
        other_side = 2 if rp.side == 1 else 1
        r_other = self.probe.probe(self.pm[other_side], rp.position)

        normal_main = r_other.n_dominant

        wall_candidates = (rp.result.n_dominant, rp.result.n_secondary)
        normal_secondary = min(
            wall_candidates, key=lambda n: abs(float(np.dot(n, normal_main)))
        )

        # A side is "on edge" only when the edge passes through this point:
        # the probe must both see a dominant jump AND straddle it roughly
        # evenly. Without the balance condition, an unrelated edge merely
        # inside the probe ball (e.g. a part boundary nearby) sets the flag
        # and misclassifies the joint type.
        def centered(r: ProbeResult) -> bool:
            return r.is_edge and 4.0 * r.balance * (1.0 - r.balance) >= 0.5

        on_edge = {rp.side: centered(rp.result), other_side: centered(r_other)}

        return SeamPoint(
            position=rp.position,
            normal_main=normal_main,
            normal_secondary=normal_secondary,
            on_edge_1=on_edge[1],
            on_edge_2=on_edge[2],
            refined_side=rp.side,
        )

    def extract_seams(self) -> List[Seam]:
        """Extract weld seams from the mesh pair and return Seam objects."""
        from .path_creator import PathCreator

        logger.info('Collecting contact-boundary seeds (CGAL)...')
        try:
            seeds = self._collect_seeds()
        except RuntimeError as e:
            logger.error(f'C++ contact extraction failed: {e}')
            return []

        if len(seeds) == 0:
            logger.warning('No contact seeds found — are the parts touching '
                           f'within epsilon={self.epsilon}m?')
            return []

        logger.info(f'{len(seeds)} seed(s); classifying (pass 1)...')

        situations = []
        for position in seeds:
            candidates = self._classify_seed(position)
            if candidates:
                situations.append((position, candidates))

        if not situations:
            logger.warning('No seed sees a geometric edge on either side')
            return []

        logger.info(
            f'{len(situations)}/{len(seeds)} seed(s) see an edge; '
            'converging to ridge (pass 2)...'
        )

        ridge: List[_RidgePoint] = []
        off_contact = 0
        for position, candidates in situations:
            for side, result in candidates:
                rp = self._walk_to_ridge(side, position, result)
                if rp is None:
                    continue
                if not self._near_contact(side, rp.position):
                    off_contact += 1
                    continue
                ridge.append(rp)
                break

        if off_contact:
            logger.debug(
                f'{off_contact} walk(s) landed on edges away from the contact '
                'zone and were rejected'
            )

        ridge = self._coherence_filter(ridge)
        ridge = self._thin(ridge)

        if len(ridge) < 2:
            logger.warning(f'Only {len(ridge)} ridge point(s) after filtering')
            return []

        logger.info(f'{len(ridge)} ridge point(s); ordering (pass 3)...')

        chains = self._order_ridge(ridge)
        if not chains:
            logger.warning('Ridge points could not be chained into seams')
            return []

        chains = self._merge_continuations(chains)
        self._extend_corners(chains)

        creator_params = dict(self.params)
        creator_params['ridge_jitter'] = self.ridge_jitter
        path_creator = PathCreator(creator_params)

        seams: List[Seam] = []
        for chain, is_closed in chains:
            seam_points = [self._build_seam_point(rp) for rp in chain]
            if len(seam_points) < 2:
                continue
            seams.extend(
                path_creator.process_path(
                    seam_points, creator_params, is_closed=is_closed
                )
            )

        logger.info(
            f'Produced {len(seams)} Seam object(s) from {len(chains)} chain(s)'
        )
        return seams
