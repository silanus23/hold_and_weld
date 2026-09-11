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

"""SeamExtractorMesh - Extract weld seams where two meshes touch.

A weld seam is the relation "one part's boundary edge lies on the other part's
surface": mark the faces within `epsilon` of the other mesh, take the edges
bounding that set, keep those that are a real part edge, chain them across both
meshes, then decide per POINT which mesh carries the edge there. Ownership is
per point because it alternates wherever a part overhangs.
"""

import logging
from typing import Dict, List, Optional, Tuple

import manifold3d
import numpy as np
from numpy.typing import NDArray
import trimesh

from .chaining import drop_coincident, loops, oriented, stitch
from .mesh_fields import MeshFields, reject_holes
from .params import SeamExtractorMeshParams
from .path_creator import PathCreator
from .seam_point import SeamPoint
from ..core.seam import Seam

logger = logging.getLogger(__name__)


class SeamExtractorMesh:
    """Extract weld seams from the contact region with per-point ownership."""

    def __init__(
        self,
        mesh_1: trimesh.Trimesh,
        mesh_2: trimesh.Trimesh,
        params: Optional[Dict] = None,
    ) -> None:
        """Initialize the contact boundary extractor.

        Args:
            mesh_1: World-frame, watertight.
            mesh_2: World-frame, watertight.
            params: Optional config dict, per SeamExtractorMeshParams in
                params.py. Unknown keys are ignored, so the same dict can be
                handed to PathCreator.

        Raises:
            ValueError: If either mesh is not watertight, has a non-finite
                vertex, or a parameter is out of range.
        """
        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')
        # Watertight says nothing about the vertex values themselves; a NaN
        # or Inf here would otherwise surface as an opaque failure deep in a
        # KDTree or manifold3d call, far from the actual cause.
        if not np.isfinite(mesh_1.vertices).all():
            raise ValueError('mesh_1 has a non-finite vertex')
        if not np.isfinite(mesh_2.vertices).all():
            raise ValueError('mesh_2 has a non-finite vertex')

        # The raw dict is kept only to hand on to PathCreator unchanged.
        self.params = params or {}
        self.cfg = SeamExtractorMeshParams.from_dict(self.params)
        self.mesh = {1: mesh_1, 2: mesh_2}

        # Every cached geometric query about the meshes themselves, and the
        # contact measure over them, lives on MeshFields; the extractor keeps
        # only what is about the SEAM.
        self.fields = MeshFields(self.mesh, self.cfg)
        self._overlap_cache: Optional[float] = None

    def interpenetration_volume(self) -> float:
        """Volume the two parts share, in m^3. Zero when they merely touch.

        Cached; the boolean costs about 0.46s on an 800k-face pair, ~3.5% of
        a mesh run. `is_watertight` does not rule out self-intersecting or
        degenerate geometry, so the boolean can still fail on real CAD input;
        that is reported and treated as unknown rather than fatal, matching
        `_report_interpenetration`'s own advisory-only stance on this check.
        """
        if self._overlap_cache is None:
            try:
                self._overlap_cache = float(
                    (self._manifold(1) ^ self._manifold(2)).volume()
                )
            except Exception as error:
                logger.warning(
                    f'Could not compute interpenetration volume: {error}. '
                    'Skipping the overlap check; the seam is still extracted.'
                )
                self._overlap_cache = 0.0
        return self._overlap_cache

    def _report_interpenetration(self) -> None:
        """Warn when the parts overlap instead of meeting at a surface.

        Reports rather than refuses. The contact boundary of a buried part is
        its buried rim, not the seam, but the failure is graded and only the
        shallow end is dangerous: measured on a 25mm cylinder into a plate at
        epsilon 2mm, 1mm buried still yields a plausible closed chain 1mm
        inside the plate, while 5mm+ buried yields no chain and so reports
        itself. Refusing outright was tried and was too brittle - a 0.1mm
        modelling interference measures 196mm^3, five orders past the
        0.001mm^3 default, yet is well inside the fit-up gap epsilon exists
        to absorb. So the operator is told what was measured and decides.
        """
        volume = self.interpenetration_volume()
        if volume > self.cfg.interpenetration_volume_m3:
            logger.warning(
                f'Parts interpenetrate by {volume * 1e9:.1f} mm^3, past the '
                f'{self.cfg.interpenetration_volume_m3 * 1e9:.3f} mm^3 '
                'threshold. Any seam found below follows the BURIED RIM '
                'rather than the surface, so check the part poses before '
                'trusting it - shallow overlap still produces a plausible '
                'closed chain at the wrong depth.'
            )

    def extract_seams(self) -> List[Seam]:
        """Extract classified Seam objects.

        Pipeline:
        1. Build ordered SeamPoint chains (`extract_chains`)
        2. Classify each chain into line/arc/PTP segments (`PathCreator`)

        Returns:
            List of Seam objects. Empty when the parts yield no chain.
        """
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

        Pipeline:
        1. Mark the faces within `epsilon` of the other mesh, per mesh, and
           take the edges bounding that set
        2. Keep the bounding edges that follow a real part edge
        3. Chain those into loops, per mesh
        4. Pool both meshes' loops and stitch them end to end across meshes
        5. Refine the points off the vertex lattice onto the contact boundary
        6. Read ownership and normals back per point

        Returns:
            List of (seam_points, is_closed) pairs. Empty when the parts do
            not touch.
        """
        self._report_interpenetration()

        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]] = {}
        touching = False
        for side in (1, 2):
            contact, boundary = self._boundary_edges(side, self.cfg.epsilon)
            touching = touching or bool(boundary)
            # Before the early returns below, so a bad epsilon is reported as
            # a bad epsilon rather than as "the parts do not touch".
            self._validate_epsilon(side, self.cfg.epsilon, boundary)
            marked[side] = (contact, self._sharp_boundary(side, boundary))
            if boundary and not marked[side][1]:
                logger.info(
                    f'mesh_{side} bounds the contact region but has no real '
                    'part edge on it; it supports the joint rather than '
                    'terminating on it'
                )

        if not touching:
            logger.warning(
                f'No contact within epsilon={self.cfg.epsilon * 1000:.2f}mm; the '
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
            for loop, is_closed in loops(side, sharp, self.cfg):
                if is_closed:
                    loop = oriented(
                        self.mesh[side], loop, contact,
                        self._wall_normal(side, loop[0], contact),
                    )
                pieces.append((self.mesh[side].vertices[loop], is_closed))

        if not pieces:
            logger.warning(
                'Contact found but neither part has a real edge on it; '
                'both surfaces are smooth across the joint'
            )
            return []

        pieces = drop_coincident(pieces, self.cfg)

        chains: List[Tuple[List[SeamPoint], bool]] = []
        for positions, is_closed in stitch(pieces, self.cfg):
            # Off the vertex lattice before anything is read off the points:
            # the attributes below are all evaluated AT a position, so they
            # have to be evaluated at the refined one.
            positions = self._refine_positions(positions, marked, is_closed)
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
        contact = self.fields.face_distance(side) <= epsilon
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

        Neither mesh is "the" seam mesh: where a part terminates its
        boundary runs along its own sharp rim, where it merely supports the
        other the boundary is an imprint across coplanar faces, and on an
        overhanging joint each mesh does both over part of the curve.
        Sharpness alone isn't enough either - a part's edges away from the
        joint are just as sharp - so a candidate must sit on the other mesh
        too.
        """
        table = self.fields.edge_dihedrals(side)
        sharp = [
            edge for edge in boundary
            if table.get(edge, 0.0) > self.cfg.edge_angle_min
        ]
        if not sharp:
            return []

        # A sharp edge is only seam where the parts actually meet. The plate's
        # outer rim is sharp along its whole length and is not a weld.
        vertices = self.mesh[side].vertices
        index = np.asarray(sharp)
        midpoints = 0.5 * (vertices[index[:, 0]] + vertices[index[:, 1]])

        distance = self.fields.surface_distance(
            2 if side == 1 else 1, midpoints)
        slack = self.cfg.near_contact_edge_fraction * self.fields.median_edge(side)
        bound = self.cfg.epsilon + slack
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
                f'{bound * 1000:.2f}mm (epsilon {self.cfg.epsilon * 1000:.2f} + '
                f'slack {slack * 1000:.2f}); distance to other mesh '
                f'median={np.median(admitted) * 1000:.3f}mm '
                f'p95={np.percentile(admitted, 95) * 1000:.3f}mm '
                f'max={admitted.max() * 1000:.3f}mm; '
                f'{int((admitted > 0.5 * bound).sum())} of them past half-bound'
            )
        return [edge for edge, ok in zip(sharp, keep) if ok]

    def _validate_epsilon(
        self,
        side: int,
        epsilon: float,
        reference: Optional[List[Tuple[int, int]]] = None,
    ) -> None:
        """Report how far the contact boundary moves when epsilon is nudged.

        Measured rather than modelled: extraction is repeated at neighbouring
        epsilon values and the boundary size compared, per direction, since
        the two failure modes differ - too small reads the parts as apart,
        too large climbs the wall. Reported as a relative change, not exact
        vertex-set equality, since a single vertex moving anywhere would
        otherwise trip it on every run of a valid config.

        Args:
            reference: The boundary at `epsilon`, when the caller already has
                it. Recomputed only when not supplied - it is a full sweep
                over the mesh's faces.
        """
        if reference is None:
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
        moved = {False: 0.0, True: 0.0}      # keyed by "probe was upward"
        for factor in self.cfg.eps_stability_factors:
            _, probe = self._boundary_edges(side, epsilon * factor)
            vertices = {v for edge in probe for v in edge}
            change = abs(len(vertices) - len(base)) / len(base)
            counts.append(
                f'{epsilon * factor * 1000:.2f}mm -> {len(vertices)} '
                f'({change * 100:+.0f}%)'
            )
            moved[factor > 1.0] = max(moved[factor > 1.0], change)

        message = f'mesh_{side} boundary vertices: ' + ', '.join(counts)
        limit = self.cfg.eps_stability_tolerance
        unstable = {up for up, change in moved.items() if change >= limit}

        # Severity is about whether a SAFE BAND exists, not about whether some
        # probe moved. Sensitivity in ONE direction means epsilon sits near
        # that end of its band - true, worth printing, but nothing to act on,
        # and warning about it every run is what teaches the reader to skip
        # these lines. BOTH directions moving means there is no band at all.
        if not unstable:
            logger.info(f'epsilon is stable within {limit * 100:.0f}% '
                        f'({message})')
        elif len(unstable) == 1:
            edge = 'upper' if unstable.pop() else 'lower'
            logger.info(
                f'epsilon={epsilon * 1000:.2f}mm sits near the {edge} end of '
                f'its band ({message}); stable the other way, so this is '
                'headroom rather than a fault'
            )
        else:
            logger.warning(
                f'epsilon={epsilon * 1000:.2f}mm has no stable band - the '
                f'contact boundary moves either way it is nudged ({message}). '
                'Too small and the parts read as apart; too large and the '
                'boundary climbs the wall instead of following the joint.'
            )

    def _refine_positions(
        self,
        positions: NDArray,
        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]],
        is_closed: bool = False,
    ) -> NDArray:
        """Move seam points off the vertex lattice onto the true contact boundary.

        Every seam point is an existing mesh vertex, so the seam is only as
        fine as the tessellation - where a part edge is much longer than the
        contact region crossing it, the true boundary falls between vertices
        and isn't in the input at all.

        `MeshFields.coverage` is 1 inside the contact, 0 outside, 1/2 on the
        boundary, so the boundary is its half level set. The field CLASSIFIES
        each point, it does not displace it: a point inside stays put, a
        point past the boundary is dropped, and only the surviving points at
        each run's ends get slid along the chain onto the level set.

        A closed chain has no first or last point, only a place the array
        happens to start, so `is_closed` carries that through to
        `reject_holes` and the slide below rather than being inferred from
        index 0 or N-1.
        """
        owner, _ = self._owner(positions)
        rho = np.zeros(len(positions))
        mating = np.zeros((len(positions), 3))
        for side in (1, 2):
            rows = np.nonzero(owner == side)[0]
            if not len(rows):
                continue
            other = 2 if side == 1 else 1
            _, nearest = self.fields.vertex_tree(side).query(positions[rows])
            _, across = self.fields.vertex_tree(other).query(positions[rows])

            # rho must span the mesh being INTEGRATED (the OTHER one), not
            # just the owner's: too small relative to ITS triangle size and
            # the centroid-sampled integral reads 0 or overshoots on luck.
            rho[rows] = self.cfg.kernel_radius_factor * np.maximum(
                self.fields.vertex_edge_scale(side)[nearest],
                self.fields.vertex_edge_scale(other)[across],
            )
            contact = marked[side][0]
            mating[rows] = [
                self._contact_normal(side, int(v), contact) for v in nearest
            ]

        coverage = np.array([
            self.fields.coverage(int(owner[i]), positions[i], rho[i], mating[i])
            for i in range(len(positions))
        ])

        # A point exactly ON the boundary reads exactly a half BY
        # CONSTRUCTION, so this is a harmless coin flip - the slide below
        # reaches the same level set either way. A margin here was tried and
        # reverted: it made one triangle's area the tolerance and fragmented
        # the seam once rho neared a triangle's size.
        inside = coverage >= 0.5

        # Rejecting nearly everything means rho or the mating normal is wrong
        # here, not that the geometry is - dropping the seam would be worse
        # than the tessellation error this exists to fix.
        if int(inside.sum()) < self.cfg.min_loop_points:
            logger.warning(
                f'Contact coverage puts only {int(inside.sum())} of '
                f'{len(positions)} seam point(s) inside the contact; leaving '
                'the chain on its mesh vertices. The mesh is probably too '
                'coarse at the joint for the half level set to exist.'
            )
            return positions

        inside = reject_holes(inside, positions, rho, is_closed)

        keep = np.nonzero(inside)[0]
        refined = positions[keep].copy()

        # Contiguous kept stretches. Their outer ends are where the contact
        # actually stops, so those are the points with a boundary to find.
        runs: List[Tuple[int, int]] = []
        start = 0
        for j in range(1, len(keep) + 1):
            if j == len(keep) or keep[j] != keep[j - 1] + 1:
                runs.append((start, j - 1))
                start = j

        # On a closed chain, if kept points reach both array ends, the first
        # run's head and the last run's tail are the same stretch seen across
        # the wrap - neither is a real contact stop, so neither slides.
        wraps = (
            is_closed and len(keep) > 0
            and keep[0] == 0 and keep[-1] == len(positions) - 1
        )
        skip = {runs[0][0], runs[-1][1]} if wraps and runs else set()

        moved = 0
        for first, last in runs:
            if last <= first:
                continue
            for outer, inner in ((first, first + 1), (last, last - 1)):
                if outer in skip:
                    continue
                direction = refined[outer] - refined[inner]
                length = float(np.linalg.norm(direction))
                if length < 1e-12:
                    continue
                landed = self.fields.slide_to_boundary(
                    refined[outer], direction / length,
                    int(owner[keep[outer]]), rho[keep[outer]],
                    mating[keep[outer]],
                )
                if landed is not None:
                    moved += 1
                    refined[outer] = landed

        dropped = len(positions) - len(keep)
        logger.info(
            f'Refined {len(positions)} seam point(s) against the contact half '
            f'level set: dropped {dropped} lying past the boundary, slid '
            f'{moved} end point(s) onto it'
        )
        return refined

    def _contact_normal(
        self, side: int, vertex: int, contact: NDArray
    ) -> NDArray:
        """Area-weighted normal of the CONTACT faces meeting a vertex.

        The mating surface, where `_wall_normal` takes the faces rising out of
        the joint. Selected by contact membership, so the two are complements
        and neither can tie.
        """
        mesh = self.mesh[side]
        incident = mesh.vertex_faces[vertex]
        incident = incident[incident >= 0]
        mating = incident[contact[incident]]
        if not len(mating):
            mating = incident

        normal = mesh.area_faces[mating] @ mesh.face_normals[mating]
        norm = np.linalg.norm(normal)
        if norm > 1e-12:
            return normal / norm
        return np.zeros(3)

    def _seam_points(
        self,
        positions: NDArray,
        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]],
    ) -> List[SeamPoint]:
        """Attach per-point ownership and normals to an ordered polyline."""
        owner, _ = self._owner(positions)
        on_edge = self._on_edge_mask(owner, positions)

        # The edge-carrying mesh supplies the wall normal, the other the base
        # surface. Both read off the owner, so neither can tie and flip.
        normal_base = np.zeros((len(positions), 3))
        normal_wall = np.zeros((len(positions), 3))
        for side in (1, 2):
            rows = np.nonzero(owner == side)[0]
            if not len(rows):
                continue
            other = self.mesh[2 if side == 1 else 1]
            base = self.fields.closest_faces(
                2 if side == 1 else 1, positions[rows])
            normal_base[rows] = other.face_normals[base]

            # From the faces NOT in contact, so it points along the surface
            # rising out of the joint rather than into it.
            contact = marked[side][0]
            _, nearest = self.fields.vertex_tree(side).query(positions[rows])
            normal_wall[rows] = [
                self._wall_normal(side, int(v), contact) for v in nearest
            ]

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

        A comparison of distances, with no "sharp enough" threshold: at a
        seam one part terminates, so one distance is essentially zero and
        the other is not. Ownership may change along a chain, which is what
        an overhang does. Also returns the distance to the winner's edge.

        A tie carries no information, and used to fall through to `<=`,
        handing every tied point to whichever mesh was `mesh_1` - order-
        dependent output, and not a rare case: on a curved mating surface the
        tessellation's own facet seams pass `edge_angle_min` too, so both
        meshes report a sharp edge underfoot.

        Ties are broken on turning DENSITY instead - comparative the same
        way, both meshes measured at one radius, no absolute angle anywhere.
        A tie that survives that is a genuine edge-to-edge joint (both parts
        really do terminate on this curve, and `_on_edge_mask` already flags
        both sides), so ownership is arbitrary in fact as well as in code and
        keeps the historical ordering rather than inventing a preference.
        """
        points = np.atleast_2d(points)
        distance = {
            side: self.fields.sharp_edge_distance(side, points) for side in (1, 2)
        }
        owner = np.where(distance[1] <= distance[2], 1, 2)

        # An ABSOLUTE gap, scaled to the mesh (as `edge_joint_floor_factor`
        # scales its floor). A relative tolerance was tried and is
        # unreachable: these distances are tiny refinement residuals, so only
        # pairs that are both exactly 0.0 would ever register as tied.
        tolerance = self.cfg.ownership_tie_factor * max(
            self.fields.median_edge(1), self.fields.median_edge(2))
        tied = (
            (np.abs(distance[1] - distance[2]) <= tolerance)
            & np.isfinite(distance[1]) & np.isfinite(distance[2])
        )
        if tied.any():
            owner[tied] = self._owner_by_turning(points[tied])

        won = np.where(owner == 1, distance[1], distance[2])
        return owner, won

    def _owner_by_turning(self, points: NDArray) -> NDArray:
        """Break a sharp-edge distance tie on which mesh turns harder."""
        radius = self._ownership_radius()
        density = {
            side: self.fields.turning_density(side, points, radius)
            for side in (1, 2)
        }

        # Neither side turning means this point is inside a contact patch,
        # not on its boundary - it shouldn't have reached ownership. Logged;
        # the tie still resolves to the historical order.
        flat = (density[1] <= 0.0) & (density[2] <= 0.0)
        if flat.any():
            logger.warning(
                f'ownership: {int(flat.sum())} of {len(points)} tied point(s) '
                f'read zero turning on BOTH meshes at radius {radius:.6f}m - '
                'no part edge on either side, check epsilon and the contact '
                'region'
            )

        logger.info(
            f'ownership: broke {len(points)} sharp-edge distance tie(s) on '
            f'turning density at radius {radius:.6f}m - '
            f'mesh 1 median={np.median(density[1]):.4f}/m '
            f'mesh 2 median={np.median(density[2]):.4f}/m'
        )
        return np.where(density[1] >= density[2], 1, 2)

    def _on_edge_mask(
        self, owner: NDArray, points: NDArray
    ) -> Dict[int, NDArray]:
        """Per-mesh, per-point comparative edge-joint test.

        Edge-to-edge is where `_owner`'s comparison is effectively a TIE -
        both parts terminating on the same curve - so the loser is judged
        against its OWN mesh's tessellation scale (`edge_joint_floor_factor *
        median_edge`) rather than the fit-up `epsilon`: this distance is a
        refinement residual, orders of magnitude smaller than epsilon is
        sized for. `| (owner == side)` is a safety fallback for a winner
        whose refined position lands just outside its own floor; on measured
        data the winner distance is already inside it.
        """
        distance = {
            side: self.fields.sharp_edge_distance(side, points) for side in (1, 2)
        }

        loser = np.where(owner == 1, distance[2], distance[1])
        if len(loser):
            logger.info(
                f'is_edge_joint: loser sharp-edge distance over '
                f'{len(loser)} point(s) - min={loser.min() * 1000:.3f}mm '
                f'median={np.median(loser) * 1000:.3f}mm '
                f'p95={np.percentile(loser, 95) * 1000:.3f}mm '
                f'max={loser.max() * 1000:.3f}mm'
            )

        return {
            side: (distance[side] <= self.cfg.edge_joint_floor_factor
                   * self.fields.median_edge(side)) | (owner == side)
            for side in (1, 2)
        }

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

        # A degenerate face has zero area AND a zero normal in trimesh, so an
        # all-degenerate (or empty) fan weights to zero rather than a direction.
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

    def _ownership_radius(self) -> float:
        """Neighbourhood radius for the turning-density comparison.

        Bounded below by the coarser mesh's own facet size - below that
        there is nothing to measure - and set by a factor, not an absolute
        length, so it follows the mesh rather than the scene units.

        Has an upper bound too: a crease's density falls as pi/(4*radius)
        while a curved wall's stays at 1/r, crossing at radius ~ 0.79*r.
        Looking WIDER weakens the discrimination, so the default factor
        stays just above the resolution floor.
        """
        return self.cfg.ownership_radius_factor * max(
            self.fields.median_edge(1), self.fields.median_edge(2))
