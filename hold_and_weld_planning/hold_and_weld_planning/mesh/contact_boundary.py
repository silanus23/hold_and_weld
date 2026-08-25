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

"""ContactBoundaryExtractor - Extract weld seams from the region where two
meshes touch.

A weld seam is the relation "one part's boundary edge lies on the other part's
surface": mark the faces within `epsilon` of the other mesh, take the edges
bounding that set, keep those that are a real part edge, chain them across both
meshes, then decide per POINT which mesh carries the edge there. Ownership is
per point because it alternates wherever a part overhangs.
"""

# TODO: (@silanus23) Solve the transition points
# Every seam point is an existing mesh vertex, so the seam is only as fine as
# the tessellation. Where a part edge is much longer than the contact region
# crossing it, the transition is simply not in the mesh — an 800mm plate rim
# carrying a 206mm chord has no vertex where the chord starts, and no tolerance
# recovers it. The crossing has to be solved and the edge split there.
#
# A second mechanism damages the same zone: the on_edge flags flicker there,
# because epsilon answers two different questions and the second is marginal
# by construction.
#
#   _sharp_boundary SELECTS edges within epsilon + slack of the other mesh
#                   (2.000 + 0.1 * 6.25 median edge = 2.625mm here)
#   _seam_points    TESTS points against those same edges at epsilon (2.000mm)
#
# The edges were chosen for sitting near 2.6mm, so the distances thresholded
# at 2.0mm cluster on the threshold. Measured on this scene:
#
#   side 1: distance to sharp edge 1.88-2.09mm across idx 57..1872,
#           26 points within +-20% of the threshold,
#           4 of 5 raw flips are +-0.1mm wobbles across the line
#   side 2: 0 points near the threshold; its 2 flips are 1.3 -> 9.5mm
#
# Only the side sharing the constant flickers. 8 contact-type flips reach
# PathCreator as 9 runs where the geometry has 2, and the min_contact_run
# absorption in _split_on_contact_type exists to glue them back together —
# the order-dependent cascade that leaves the 1.3mm and 6.9mm stubs.
#
# Sweeping epsilon cannot fix it: selection and test move together, which is
# why the seam count held at 8 across the near_contact_edge_fraction sweep in
# the config. The fix is to stop sharing the constant — give the on_edge test
# its own threshold above the selection bound — and only then smooth by arc
# length if anything still flickers. Hysteresis is the wrong tool: a seam has
# no canonical traversal direction.
#
# The two are independent: inserting the missing crossing point does not make
# the flags less marginal, and separating the thresholds does not create the
# point. Either alone shrinks the stubs; both are needed to remove them.

# TODO: (@silanus23) Weigh ownership by sharpness, not proximity alone
# _owner picks whichever mesh has the NEAREST sharp edge, and every edge above
# edge_angle_min counts equally. A 2.81 deg facet seam on a tessellated cylinder
# therefore outranks a real 90 deg rim whenever it happens to be marginally
# closer. Suspect this degrades the transition zones, where both meshes have
# edges within a millimetre of the seam and proximity alone cannot separate them.

import logging
from typing import Dict, List, Optional, Tuple

import manifold3d
import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from .path_creator import PathCreator
from .seam_point import SeamPoint
from ..core.seam import Seam

logger = logging.getLogger(__name__)


class ContactBoundaryExtractor:
    """Extract weld seams from the contact region with per-point ownership.

    Args:
        mesh_1: First mesh (world frame, watertight).
        mesh_2: Second mesh (world frame, watertight).
        params: Configuration dict; see PARAMS.md.

    Raises:
        ValueError: If either mesh is not watertight.
    """

    def __init__(
        self,
        mesh_1: trimesh.Trimesh,
        mesh_2: trimesh.Trimesh,
        params: Optional[Dict] = None,
    ) -> None:
        """Initialize the extractor; see the class docstring for arguments."""
        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')

        self.params = params or {}
        self.epsilon = float(self.params.get('epsilon', 0.002))
        self.edge_angle_min = np.radians(
            float(self.params.get('edge_angle_min_deg', 0.0057)))
        self.near_contact_fraction = float(
            self.params.get('near_contact_edge_fraction', 0.1))
        self.stitch_gap_factor = float(
            self.params.get('stitch_gap_factor', 3.0))
        self.interpenetration_volume = float(
            self.params.get('interpenetration_volume_m3', 1e-12))
        factors = self.params.get('eps_stability_factors', (0.75, 1.5))
        try:
            self.eps_stability_factors = tuple(float(f) for f in factors)
        except (TypeError, ValueError):
            raise ValueError(
                'eps_stability_factors must be a list of probe factors, '
                f'e.g. [0.75, 1.5]; got {factors!r}'
            )
        self.closest_face_candidates = int(
            self.params.get('closest_face_candidates', 12))
        self.closest_vertex_candidates = int(
            self.params.get('closest_vertex_candidates', 4))
        self.min_loop_points = int(self.params.get('min_loop_points', 4))
        self.sharp_edge_max_samples = int(
            self.params.get('sharp_edge_max_samples', 256))
        self._validate_params()
        self.mesh = {1: mesh_1, 2: mesh_2}

        self._distance_cache: Dict[int, NDArray] = {}
        self._dihedral_cache: Dict[int, Dict[Tuple[int, int], float]] = {}
        self._sharp_tree_cache: Dict[int, Optional[KDTree]] = {}
        self._route_cache: Optional[str] = None

    def _validate_params(self) -> None:
        """Reject parameter values that fail late, or silently reassure.

        Every failure caught here is a misleading one rather than a loud one:
        a non-positive epsilon marks no face as contact and is reported as
        'the parts do not touch', a stability factor of exactly 1.0 probes
        epsilon against itself and always reports a stable plateau, and a
        min_loop_points below 3 reaches _oriented as an index error on a loop
        that was never a loop.
        """
        for key, value in (
            ('epsilon', self.epsilon),
            ('closest_face_candidates', self.closest_face_candidates),
            ('closest_vertex_candidates', self.closest_vertex_candidates),
        ):
            if not value > 0:
                raise ValueError(f'{key} must be > 0, got {value}')

        # Zero is a legitimate way to disable each of these, negative is not.
        # edge_angle_min is reported in the degrees the user wrote, not the
        # radians it is stored as.
        for key, value in (
            ('edge_angle_min_deg', float(np.degrees(self.edge_angle_min))),
            ('near_contact_edge_fraction', self.near_contact_fraction),
            ('stitch_gap_factor', self.stitch_gap_factor),
            ('interpenetration_volume_m3', self.interpenetration_volume),
        ):
            if value < 0.0:
                raise ValueError(f'{key} must be >= 0, got {value}')

        # _oriented reads loop[1] and loop[-1]; below 3 there is no loop.
        if self.min_loop_points < 3:
            raise ValueError(
                f'min_loop_points must be >= 3, got {self.min_loop_points}')

        # `steps` in _sharp_edge_tree has a floor of 2, so a smaller cap would
        # not cap anything.
        if self.sharp_edge_max_samples < 2:
            raise ValueError(
                'sharp_edge_max_samples must be >= 2, got '
                f'{self.sharp_edge_max_samples}')

        if not self.eps_stability_factors:
            raise ValueError(
                'eps_stability_factors must name at least one probe factor; '
                'with none, the plateau check has nothing to compare against '
                'and reports every epsilon as stable'
            )
        for factor in self.eps_stability_factors:
            if not factor > 0.0:
                raise ValueError(
                    f'eps_stability_factors must all be > 0, got {factor}')
            if factor == 1.0:
                raise ValueError(
                    'eps_stability_factors of exactly 1.0 probes epsilon '
                    'against itself and always reports a stable plateau'
                )

    def route(self) -> str:
        """'contact' when the parts meet at a surface, 'intersect' when one is
        buried in the other."""
        if self._route_cache is not None:
            return self._route_cache

        volume = float(
            (self._manifold(1) ^ self._manifold(2)).volume()
        )
        if volume > self.interpenetration_volume:
            logger.info(
                f'Parts interpenetrate ({volume * 1e9:.1f} mm^3 of overlap); '
                'the contact boundary is the buried rim, not the seam'
            )
            self._route_cache = 'intersect'
        else:
            self._route_cache = 'contact'
        return self._route_cache

    def extract_seams(self) -> List[Seam]:
        """Extract classified Seam objects."""
        chains = self.extract_chains()
        if not chains:
            return []

        creator_params = dict(self.params)
        path_creator = PathCreator(creator_params)

        seams: List[Seam] = []
        for points, is_closed in chains:
            if len(points) < 2:
                continue
            seams.extend(
                path_creator.process_path(
                    points, creator_params, is_closed=is_closed
                )
            )

        logger.info(
            f'Produced {len(seams)} Seam object(s) from {len(chains)} loop(s)'
        )
        return seams

    def extract_chains(self) -> List[Tuple[List[SeamPoint], bool]]:
        """Build ordered SeamPoint chains with per-point edge ownership.

        Returns:
            List of (seam_points, is_closed) pairs. Empty when the parts do not
            touch, or interpenetrate and need the intersection path instead.
        """
        if self.route() == 'intersect':
            return []

        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]] = {}
        touching = False
        for side in (1, 2):
            contact, boundary = self._boundary_edges(side, self.epsilon)
            touching = touching or bool(boundary)
            # Before the early returns below, so a bad epsilon is reported as
            # a bad epsilon rather than as "the parts do not touch".
            self._validate_epsilon(side, self.epsilon)
            marked[side] = (contact, self._sharp_boundary(side, boundary))
            if boundary and not marked[side][1]:
                logger.info(
                    f'mesh_{side} bounds the contact region but has no real '
                    'part edge on it; it supports the joint rather than '
                    'terminating on it'
                )

        if not touching:
            logger.warning(
                f'No contact within epsilon={self.epsilon * 1000:.2f}mm; the '
                'parts do not touch at this fit-up'
            )
            return []

        # Pool both meshes' pieces, then stitch across meshes. This drops which
        # mesh a piece came from; ownership is read back per point afterwards.
        pieces: List[Tuple[NDArray, bool]] = []
        for side in (1, 2):
            contact, sharp = marked[side]
            if not sharp:
                continue
            for loop, is_closed in self._loops(side, sharp):
                if is_closed:
                    loop = self._oriented(side, loop, contact)
                pieces.append((self.mesh[side].vertices[loop], is_closed))

        if not pieces:
            logger.warning(
                'Contact found but neither part has a real edge on it; '
                'both surfaces are smooth across the joint'
            )
            return []

        pieces = self._drop_coincident(pieces)

        chains: List[Tuple[List[SeamPoint], bool]] = []
        for positions, is_closed in self._stitch(pieces):
            if len(positions) < 2:
                continue
            chains.append((self._seam_points(positions, marked), is_closed))

        total = sum(len(points) for points, _ in chains)
        owners = [p.owner_side for points, _ in chains for p in points]
        switches = sum(
            1 for points, _ in chains
            for a, b in zip(points, points[1:])
            if a.owner_side != b.owner_side
        )
        logger.info(
            f'{len(chains)} seam chain(s), {total} seam point(s); ownership '
            f'mesh_1={owners.count(1)} mesh_2={owners.count(2)}, '
            f'{switches} in-chain ownership change(s)'
        )
        return chains

    def _boundary_edges(
        self, side: int, epsilon: float
    ) -> Tuple[NDArray, List[Tuple[int, int]]]:
        """Contact mask and the edges with exactly one incident marked face."""
        contact = self._face_distance(side) <= epsilon
        incident: Dict[Tuple[int, int], int] = {}
        faces = self.mesh[side].faces

        for index in np.nonzero(contact)[0]:
            v0, v1, v2 = faces[index]
            for a, b in ((v0, v1), (v1, v2), (v2, v0)):
                key = (int(min(a, b)), int(max(a, b)))
                incident[key] = incident.get(key, 0) + 1

        boundary = [key for key, count in incident.items() if count == 1]
        return contact, boundary

    def _sharp_boundary(
        self, side: int, boundary: List[Tuple[int, int]]
    ) -> List[Tuple[int, int]]:
        """Contact-boundary edges that follow a real part edge of this mesh.

        Both meshes bound the contact region, but neither is "the" seam mesh:
        where a part terminates its boundary runs along its own sharp rim,
        where it merely supports the other the boundary is an imprint across
        coplanar faces, and on an overhanging joint each mesh does both over
        part of the curve. Sharpness alone is not enough either — a part's
        edges away from the joint are just as sharp — so a candidate must sit
        on the other mesh too.
        """
        table = self._edge_dihedrals(side)
        sharp = [
            edge for edge in boundary
            if table.get(edge, 0.0) > self.edge_angle_min
        ]
        if not sharp:
            return []

        # A sharp edge is only seam where the parts actually meet. The plate's
        # outer rim is sharp along its whole length and is not a weld.
        vertices = self.mesh[side].vertices
        index = np.asarray(sharp)
        midpoints = 0.5 * (vertices[index[:, 0]] + vertices[index[:, 1]])

        other = self.mesh[2 if side == 1 else 1]
        distance = self._surface_distance(other, midpoints)
        slack = self.near_contact_fraction * self._median_edge(side)
        bound = self.epsilon + slack
        keep = distance <= bound

        dropped = int((~keep).sum())
        if dropped:
            logger.info(
                f'mesh_{side}: dropped {dropped} of {len(sharp)} sharp '
                'boundary edge(s) that are real part edges away from the joint'
            )

        # A candidate admitted at nearly the full bound is a real part edge
        # that is NOT a weld (an overhanging rim); those sit just inside the
        # bound, not near zero.
        if keep.any():
            admitted = distance[keep]
            logger.info(
                f'mesh_{side}: {int(keep.sum())} edge(s) admitted at bound '
                f'{bound * 1000:.2f}mm (epsilon {self.epsilon * 1000:.2f} + '
                f'slack {slack * 1000:.2f}); distance to other mesh '
                f'median={np.median(admitted) * 1000:.3f}mm '
                f'p95={np.percentile(admitted, 95) * 1000:.3f}mm '
                f'max={admitted.max() * 1000:.3f}mm; '
                f'{int((admitted > 0.5 * bound).sum())} of them past half-bound'
            )
        return [edge for edge, ok in zip(sharp, keep) if ok]

    def _validate_epsilon(self, side: int, epsilon: float) -> None:
        """Warn when epsilon is not on the plateau where the boundary is fixed.

        Measured rather than modelled: extraction is repeated at neighbouring
        epsilon values and the boundary vertex set compared. On the plateau the
        set is identical.
        """
        _, reference = self._boundary_edges(side, epsilon)
        base = {v for edge in reference for v in edge}
        if not base:
            logger.warning(
                f'mesh_{side}: no contact boundary at epsilon='
                f'{epsilon * 1000:.2f}mm; too small for the fit-up gap, or the '
                'parts do not meet'
            )
            return

        counts = [f'{epsilon * 1000:.2f}mm -> {len(base)}']
        stable = True

        for factor in self.eps_stability_factors:
            _, probe = self._boundary_edges(side, epsilon * factor)
            vertices = {v for edge in probe for v in edge}
            counts.append(f'{epsilon * factor * 1000:.2f}mm -> {len(vertices)}')
            if vertices != base:
                stable = False

        message = f'mesh_{side} boundary vertices: ' + ', '.join(counts)
        if stable:
            logger.info(f'epsilon is on the stable plateau ({message})')
        else:
            logger.warning(
                f'epsilon={epsilon * 1000:.2f}mm is NOT on a stable plateau '
                f'({message}). Below the fit-up gap the parts read as apart; '
                'above ~half the transverse face size the boundary climbs the '
                'wall. Re-check epsilon against MESH_MATH_REVIEW.md 0.6.'
            )

    def _loops(
        self, side: int, boundary: List[Tuple[int, int]]
    ) -> List[Tuple[List[int], bool]]:
        """Walk boundary edges into ordered chains of vertex indices.

        A boundary vertex normally joins exactly two boundary edges, so the
        walk has one way to continue and comes back to where it started. Degree
        above two means two boundary curves cross — the arc/chord corner where
        one part overhangs — and nothing local says which branch continues the
        same seam. The walk stops at such a junction rather than guessing, so
        the curves through it come out as open chains instead of one spliced
        loop.
        """
        adjacency: Dict[int, List[int]] = {}
        for a, b in boundary:
            adjacency.setdefault(a, []).append(b)
            adjacency.setdefault(b, []).append(a)

        junctions = {v for v, n in adjacency.items() if len(n) != 2}
        if junctions:
            logger.info(
                f'mesh_{side} contact boundary has {len(junctions)} junction '
                f'vertex(es) of degree != 2; cutting into open chains there'
            )

        chains: List[Tuple[List[int], bool]] = []
        used: set = set()

        def walk(start: int, first: int) -> List[int]:
            chain = [start, first]
            used.add((min(start, first), max(start, first)))
            previous, current = start, first
            while current not in junctions:
                options = [v for v in adjacency[current] if v != previous]
                if not options:
                    break
                nxt = options[0]
                key = (min(current, nxt), max(current, nxt))
                if key in used:
                    break
                used.add(key)
                chain.append(nxt)
                previous, current = current, nxt
            return chain

        # Junctions first: each branch leaving one is a separate open chain.
        for junction in junctions:
            for neighbour in adjacency[junction]:
                key = (min(junction, neighbour), max(junction, neighbour))
                if key not in used:
                    chains.append((walk(junction, neighbour), False))

        # Whatever is left touches no junction, so it can only be a closed loop.
        for start in adjacency:
            for neighbour in adjacency[start]:
                key = (min(start, neighbour), max(start, neighbour))
                if key in used:
                    continue
                chain = walk(start, neighbour)
                closed = len(chain) > 2 and chain[-1] == chain[0]
                if closed:
                    chain = chain[:-1]
                chains.append((chain, closed))

        return [
            (chain, closed)
            for chain, closed in chains
            if len(chain) >= self.min_loop_points
        ]

    def _oriented(
        self, side: int, loop: List[int], contact: NDArray
    ) -> List[int]:
        """Orient a loop so the contact region lies consistently to one side."""
        mesh = self.mesh[side]
        positions = mesh.vertices[loop]
        centroid = mesh.triangles_center[contact].mean(axis=0)

        tangent = positions[1] - positions[-1]
        norm = np.linalg.norm(tangent)
        if norm < 1e-12:
            return loop
        tangent /= norm

        inward = np.cross(tangent, self._wall_normal(side, loop[0], contact))
        if np.dot(inward, centroid - positions[0]) < 0.0:
            return [loop[0]] + loop[1:][::-1]
        return loop

    def _drop_coincident(
        self, pieces: List[Tuple[NDArray, bool]]
    ) -> List[Tuple[NDArray, bool]]:
        """Drop a piece that lies on top of a better-sampled one.

        On an edge-to-edge joint both parts terminate on the same curve and it
        would be welded twice. Coincident pieces sit within epsilon along their
        whole length — the same condition that defined contact, so no new
        tolerance is introduced.
        """
        keep: List[Tuple[NDArray, bool]] = []
        for index, (positions, is_closed) in enumerate(pieces):
            redundant = False
            for other_index, (others, _) in enumerate(pieces):
                if other_index == index or len(others) < len(positions):
                    continue
                if len(others) == len(positions) and other_index > index:
                    continue
                covered = KDTree(others).query(positions)[0].max()
                if float(covered) <= self.epsilon:
                    redundant = True
                    break
            if redundant:
                logger.info(
                    f'Dropping {len(positions)}-point piece: another piece '
                    'samples the same curve more finely'
                )
            else:
                keep.append((positions, is_closed))
        return keep

    def _stitch(
        self, pieces: List[Tuple[NDArray, bool]]
    ) -> List[Tuple[NDArray, bool]]:
        """Join polylines from different meshes end to end into one chain.

        Each mesh contributes only the portion of the seam where it terminates,
        and the two share no vertices, so the pieces meet with a gap of roughly
        one segment. Closing those gaps gives one chain whose ownership
        alternates along its length.

        """
        open_pieces = [list(p) for p, closed in pieces if not closed]
        result = [(p, True) for p, closed in pieces if closed]

        def span(piece: List[NDArray]) -> float:
            """Local sampling density of one piece."""
            if len(piece) < 2:
                return 0.0
            return float(np.median(
                np.linalg.norm(np.diff(np.asarray(piece), axis=0), axis=1)
            ))

        # No early return for a lone open piece: the merge loop below already
        # no-ops on one piece, and that piece still has to reach the closure
        # test — a seam that closes on itself needs no second piece to do it.

        # Tolerance comes from the two pieces being joined, never pooled across
        # all of them: the meshes are sampled at wildly different densities, so
        # a pooled median is dominated by the finer one and would refuse every
        # real handoff.
        merged = True
        while merged and len(open_pieces) > 1:
            merged = False
            for i in range(len(open_pieces)):
                for j in range(len(open_pieces)):
                    if i == j:
                        continue
                    a, b = open_pieces[i], open_pieces[j]
                    options = (
                        (np.linalg.norm(a[-1] - b[0]), False, False),
                        (np.linalg.norm(a[-1] - b[-1]), False, True),
                        (np.linalg.norm(a[0] - b[0]), True, False),
                        (np.linalg.norm(a[0] - b[-1]), True, True),
                    )
                    gap, flip_a, flip_b = min(options, key=lambda o: o[0])
                    tolerance = self.stitch_gap_factor * max(span(a), span(b))
                    if gap > tolerance:
                        continue
                    logger.info(
                        f'Stitched a {len(a)}-point and a {len(b)}-point piece; '
                        f'gap {gap * 1000:.2f}mm within tolerance '
                        f'{tolerance * 1000:.2f}mm'
                    )
                    head = a[::-1] if flip_a else a
                    tail = b[::-1] if flip_b else b
                    open_pieces[i] = head + tail
                    open_pieces.pop(j)
                    merged = True
                    break
                if merged:
                    break

        for piece in open_pieces:
            array = np.asarray(piece)
            closed = (
                len(array) >= self.min_loop_points
                and float(np.linalg.norm(array[0] - array[-1]))
                <= self.stitch_gap_factor * span(piece)
            )
            if closed:
                array = array[:-1]
            result.append((array, closed))
        return result

    def _seam_points(
        self,
        positions: NDArray,
        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]],
    ) -> List[SeamPoint]:
        """Attach per-point ownership and normals to an ordered polyline."""
        owner, _ = self._owner(positions)

        # The edge-carrying mesh supplies the wall normal, the other the base
        # surface. Both read off the owner, so neither can tie and flip.
        normal_base = np.zeros((len(positions), 3))
        normal_wall = np.zeros((len(positions), 3))
        for side in (1, 2):
            rows = np.nonzero(owner == side)[0]
            if not len(rows):
                continue
            other = self.mesh[2 if side == 1 else 1]
            base = self._closest_faces(other, positions[rows])
            normal_base[rows] = other.face_normals[base]

            # From the faces NOT in contact, so it points along the surface
            # rising out of the joint rather than into it.
            contact = marked[side][0]
            _, nearest = KDTree(self.mesh[side].vertices).query(positions[rows])
            normal_wall[rows] = [
                self._wall_normal(side, int(v), contact) for v in nearest
            ]

        # Edge-to-edge where BOTH parts terminate; edge-to-surface otherwise.
        on_edge = {}
        for side in (1, 2):
            tree = self._sharp_edge_tree(side)
            near = (
                tree.query(positions)[0] <= self.epsilon
                if tree is not None
                else np.zeros(len(positions), dtype=bool)
            )
            on_edge[side] = np.asarray(near) | (owner == side)

        return [
            SeamPoint(
                position=positions[i],
                normal_base=normal_base[i],
                normal_wall=normal_wall[i],
                on_edge_1=bool(on_edge[1][i]),
                on_edge_2=bool(on_edge[2][i]),
                owner_side=int(owner[i]),
            )
            for i in range(len(positions))
        ]

    def _owner(self, points: NDArray) -> Tuple[NDArray, NDArray]:
        """Which mesh carries the geometric edge at each point, 1 or 2.

        A comparison of distances, with no "sharp enough" threshold: at a seam
        one part terminates, so one distance is essentially zero and the other
        is not. Ownership may change along a single chain, which is what an
        overhang does. Also returns the distance to the winner's sharp edge.
        """
        points = np.atleast_2d(points)
        distance = {}
        for side in (1, 2):
            tree = self._sharp_edge_tree(side)
            distance[side] = (
                tree.query(points)[0] if tree is not None
                else np.full(len(points), np.inf)
            )
        owner = np.where(distance[1] <= distance[2], 1, 2)
        won = np.where(owner == 1, distance[1], distance[2])
        return owner, won

    def _wall_normal(self, side: int, vertex: int, contact: NDArray) -> NDArray:
        """Area-weighted normal of the non-contact faces meeting a vertex.

        These are the faces rising away from the joint. Selected by contact
        membership, not an area majority, so there is no label left to tie.
        """
        mesh = self.mesh[side]
        incident = mesh.vertex_faces[vertex]
        incident = incident[incident >= 0]
        wall = incident[~contact[incident]]
        if not len(wall):
            wall = incident

        # A degenerate face carries zero area AND a zero normal in trimesh, so
        # an all-degenerate fan weights to zero rather than to a direction; so
        # does an empty fan, at a vertex no face references.
        normal = mesh.area_faces[wall] @ mesh.face_normals[wall]
        norm = np.linalg.norm(normal)
        if norm > 1e-12:
            return normal / norm

        normal = mesh.vertex_normals[vertex]
        norm = np.linalg.norm(normal)
        if norm > 1e-12:
            return normal / norm

        logger.warning(
            f'mesh_{side}: vertex {vertex} has no usable wall normal; '
            'emitting a zero normal, the weld pose there will be unusable'
        )
        return np.zeros(3)

    def _manifold(self, side: int) -> manifold3d.Manifold:
        """Convert one mesh to a manifold3d solid for boolean tests."""
        mesh = self.mesh[side]
        data = manifold3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
        )
        return manifold3d.Manifold(data)

    def _surface_distance(
        self, mesh: trimesh.Trimesh, points: NDArray
    ) -> NDArray:
        """Exact distance from each point to `mesh`'s surface."""
        points = np.atleast_2d(points)
        faces = []

        count = min(self.closest_face_candidates, len(mesh.faces))
        _, by_centroid = KDTree(mesh.triangles_center).query(points, k=count)
        faces.append(by_centroid.reshape(len(points), count))

        vcount = min(self.closest_vertex_candidates, len(mesh.vertices))
        _, near_vertices = KDTree(mesh.vertices).query(points, k=vcount)
        near_vertices = near_vertices.reshape(len(points), vcount)
        incident = mesh.vertex_faces[near_vertices]
        faces.append(np.where(incident < 0, by_centroid[:, :1, None], incident)
                     .reshape(len(points), -1))

        candidates = np.concatenate(faces, axis=1)
        distance = np.full(len(points), np.inf)
        for column in range(candidates.shape[1]):
            closest = trimesh.triangles.closest_point(
                mesh.triangles[candidates[:, column]], points
            )
            distance = np.minimum(
                distance, np.linalg.norm(closest - points, axis=1)
            )
        return distance

    def _closest_faces(self, mesh: trimesh.Trimesh, points: NDArray) -> NDArray:
        """Index of the nearest face of `mesh` for each point."""
        points = np.atleast_2d(points)
        count = min(self.closest_face_candidates, len(mesh.faces))
        _, by_centroid = KDTree(mesh.triangles_center).query(points, k=count)
        by_centroid = by_centroid.reshape(len(points), count)

        vcount = min(self.closest_vertex_candidates, len(mesh.vertices))
        _, near_vertices = KDTree(mesh.vertices).query(points, k=vcount)
        incident = mesh.vertex_faces[near_vertices.reshape(len(points), vcount)]
        incident = np.where(
            incident < 0, by_centroid[:, :1, None], incident
        ).reshape(len(points), -1)
        candidates = np.concatenate([by_centroid, incident], axis=1)

        best = np.zeros(len(points), dtype=np.int64)
        best_distance = np.full(len(points), np.inf)
        for column in range(candidates.shape[1]):
            faces = candidates[:, column]
            closest = trimesh.triangles.closest_point(
                mesh.triangles[faces], points
            )
            distance = np.linalg.norm(closest - points, axis=1)
            better = distance < best_distance
            best_distance = np.where(better, distance, best_distance)
            best = np.where(better, faces, best)
        return best

    def _sharp_edge_tree(self, side: int) -> Optional[KDTree]:
        """KD-tree over points sampled along every sharp edge of one mesh.

        Ownership is a distance comparison against these, so sampling must be
        dense enough that a long sharp edge is not represented by its endpoints.
        """
        if side in self._sharp_tree_cache:
            return self._sharp_tree_cache[side]

        mesh = self.mesh[side]
        table = mesh.face_adjacency_angles
        edges = mesh.face_adjacency_edges[table > self.edge_angle_min]
        if not len(edges):
            self._sharp_tree_cache[side] = None
            return None

        start, end = mesh.vertices[edges[:, 0]], mesh.vertices[edges[:, 1]]
        longest = float(np.linalg.norm(end - start, axis=1).max())
        steps = max(2, int(np.ceil(longest / max(self.epsilon, 1e-9))) + 1)
        steps = min(steps, self.sharp_edge_max_samples)
        t = np.linspace(0.0, 1.0, steps)[:, None, None]
        samples = (start[None] * (1.0 - t) + end[None] * t).reshape(-1, 3)

        tree = KDTree(samples)
        self._sharp_tree_cache[side] = tree
        return tree

    def _median_edge(self, side: int) -> float:
        """Median edge length of one mesh."""
        mesh = self.mesh[side]
        return float(np.median(np.linalg.norm(
            mesh.vertices[mesh.edges_unique[:, 0]]
            - mesh.vertices[mesh.edges_unique[:, 1]], axis=1)))

    def _face_distance(self, side: int) -> NDArray:
        """Distance from each face centroid of `side` to the other surface."""
        if side in self._distance_cache:
            return self._distance_cache[side]
        other = self.mesh[2 if side == 1 else 1]
        distance = self._surface_distance(
            other, self.mesh[side].triangles_center
        )
        self._distance_cache[side] = distance
        return distance

    def _edge_dihedrals(self, side: int) -> Dict[Tuple[int, int], float]:
        """Map each interior mesh edge to the angle between its two faces."""
        if side in self._dihedral_cache:
            return self._dihedral_cache[side]

        mesh = self.mesh[side]
        table = {
            (int(min(a, b)), int(max(a, b))): float(angle)
            for (a, b), angle in zip(
                mesh.face_adjacency_edges, mesh.face_adjacency_angles
            )
        }
        self._dihedral_cache[side] = table
        return table

    # ---------------------------------------------------- retired with ridge
    # Kept for the transition-points TODO at the top of this file: the only
    # corner reconstruction this package has had. It takes two nearby OPEN
    # chain ends, extends both end tangents, and solves the skew-line closest
    # approach for where they would meet. Measured on the current scene,
    # intersecting the fitted arc with the plate's rim line places the corner
    # within 0.10mm, against the 3-6mm the stubs sit at now.
    #
    # Three ridge-era dependencies need substituting to rewire it here:
    #   probe_radius        -> a length scale from this class (median edge)
    #   _project_to_surface -> trimesh closest_point on the owning mesh
    #   _RidgePoint.side    -> SeamPoint.owner_side
    #
    # def _end_tangent(self, chain, at_head):
    #     """Return (end_position, outward unit tangent) of one chain end."""
    #     pts = chain[:5] if at_head else chain[-5:]
    #     local = np.array([p.position for p in pts])
    #     centroid = local.mean(axis=0)
    #     _, _, vt = np.linalg.svd(local - centroid, full_matrices=False)
    #     tangent = vt[0]
    #     end_pos = local[0] if at_head else local[-1]
    #     if np.dot(end_pos - centroid, tangent) < 0.0:
    #         tangent = -tangent
    #     return end_pos, tangent
    #
    # def _extend_corners(self, chains):
    #     """Reconstruct corner points the extractor cannot see, in place."""
    #     ends = []
    #     for ci, (chain, is_closed) in enumerate(chains):
    #         if is_closed or len(chain) < 3:
    #             continue
    #         for at_head in (True, False):
    #             ends.append((ci, at_head) + self._end_tangent(chain, at_head))
    #
    #     pairs = []
    #     for i in range(len(ends)):
    #         for j in range(i + 1, len(ends)):
    #             if ends[i][0] == ends[j][0]:
    #                 continue            # never bridge a chain to itself
    #             gap = float(np.linalg.norm(ends[i][2] - ends[j][2]))
    #             if gap <= radius:
    #                 pairs.append((gap, i, j, radius))
    #     pairs.sort()
    #
    #     used = set()
    #     for gap, i, j, radius in pairs:
    #         if i in used or j in used:
    #             continue
    #         _, _, p1, t1 = ends[i]
    #         _, _, p2, t2 = ends[j]
    #         # Closest point between p = p1 + s*t1 and q = p2 + u*t2.
    #         cross = np.cross(t1, t2)
    #         denom = float(np.dot(cross, cross))
    #         if denom < 1e-10:
    #             continue                # near-parallel ends: not a corner
    #         w = p2 - p1
    #         s = float(np.dot(np.cross(w, t2), cross)) / denom
    #         u = float(np.dot(np.cross(w, t1), cross)) / denom
    #         if s <= 0.0 or u <= 0.0:
    #             continue                # intersection behind an end
    #         corner = 0.5 * ((p1 + s * t1) + (p2 + u * t2))
    #         if (np.linalg.norm(corner - p1) > radius
    #                 or np.linalg.norm(corner - p2) > radius):
    #             continue
    #         # A straight tangent extended from a curved chain leaves the
    #         # surface (chord error); pull the corner back onto the mesh.
    #         corner = project_to_surface(corner)
    #         used.update((i, j))
    #         # then insert `corner` at the head/tail of both chains
