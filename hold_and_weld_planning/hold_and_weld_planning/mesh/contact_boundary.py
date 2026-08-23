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

"""Weld seam extraction from the contact region, with per-point edge ownership.

A weld seam is not a property of either part alone: it is the relation "one
part's boundary edge lies on the other part's surface". Where two parts meet
flush, one of them is typically *flat* at the joint, so a single-mesh crease
detector has nothing to find there. This module computes the relation itself:

    1. mark each face of a mesh whose centroid lies within `epsilon` of the
       other mesh;
    2. candidate seam = every mesh edge with exactly ONE incident marked face;
    3. keep the ones that are a real part edge AND sit on the other mesh;
    4. chain them, stitching ACROSS meshes into single ordered chains;
    5. for each point, decide which mesh carries the edge THERE.

Step 5 is the load-bearing one. At a joint one part *terminates* (it has a
real edge) and the other *supports* (it is locally flat), but which part does
which is not constant along a seam: where a part overhangs, the overhanging
part owns the arc and the supported part owns the chord along its own rim.
Ownership therefore has to be per POINT.

An earlier version of this module resolved ownership per CHAIN — every point
of a chain came from one mesh by construction. That is what made a joint with
alternating ownership come apart into disconnected pieces with gaps of one
mesh edge at every handoff (measured 14-58mm). Ownership is now read back
after chaining rather than used to route points into chains, so a single chain
may change owner along its length.

Ownership itself is a comparison, not a threshold: whichever mesh has the
nearer sharp edge at that point owns it. At a real seam one distance is
essentially zero and the other is not, so there is nothing to tune, and no
label tie of the kind that flips a torch by 180 degrees on an arc.

Two constraints are physical, measured, and asserted rather than tuned:

    fit-up gap  <  epsilon  <  ~half the transverse face size

Below the lower bound the parts register as not touching; above the upper
bound the marked set creeps up the transverse wall and the boundary climbs
with it.

    (mesh_1 AND mesh_2).volume() > 0  =>  the parts interpenetrate

Interpenetrating parts have no contact boundary in the sense used here: the
boundary of the marked set is the buried rim, not the curve where the parts'
surfaces cross. Those must be routed to the exact corefinement intersection
path instead; `route()` reports which case applies.

KNOWN LIMIT — coarse meshes. Every seam point is an existing mesh vertex, so
the seam can only be as fine as the tessellation. Where a part's edge is much
longer than the contact region it crosses, the transition point simply is not
in the mesh and no tolerance recovers it: an 800mm plate rim carrying a 206mm
chord has no vertex where the chord starts. Fixing that needs the crossing to
be SOLVED and the edge split there. See MESH_MATH_REVIEW.md section 0.8.

See MESH_MATH_REVIEW.md sections 0.6 and 0.8 for the measurements behind all
of this.
"""

import logging
from typing import Dict, List, Optional, Tuple

import manifold3d
import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from .path_creator import PathCreator
from .seam_extractor import SeamPoint
from ..core.seam import Seam

logger = logging.getLogger(__name__)

# Below this the two adjacent faces of a mesh edge count as coplanar, i.e.
# the edge is a triangulation artefact rather than a real part edge. This is a
# numerical floor separating 0 from 90 degrees, not a tuning parameter.
_COPLANAR_RAD = 1e-4

# Interpenetration test floor, in cubic metres. Manifold returns exactly -0.0
# for flush contact and 3.8e-5 m^3 for half a millimetre of overlap, so this
# only absorbs floating-point noise.
_INTERPENETRATION_VOLUME = 1e-12

# Epsilon is validated by re-running the extraction at these multiples and
# checking the boundary does not move: a direct measurement of whether we are
# sitting on the plateau, instead of a model of where the plateau is.
_EPS_STABILITY_FACTORS = (0.75, 1.5)

# Candidate faces pulled from the KD-tree before exact point-triangle tests.
# Centroid proximity alone is not enough: a fan-triangulated cap (a cylinder
# end disc) has long thin triangles whose centroids sit far from the points
# they contain, so the containing triangle can be missed entirely and a point
# lying exactly ON the surface reads as tens of millimetres away. Faces
# incident to the nearest VERTICES are therefore added to the candidate set.
_CLOSEST_CANDIDATES = 12
_CLOSEST_VERTICES = 4

# A converged seam point must sit essentially on the other mesh: a weld seam
# exists only where the parts meet. Without this, a mesh's own part edges far
# from the joint (a plate's outer rim) are perfectly sharp and are admitted as
# seam. The tessellation slack matches SeamExtractor._near_contact.
#
# Overridable as 'near_contact_edge_fraction'. NOTE this is a fraction of the
# mesh's MEDIAN edge length, so it is only meaningful when the mesh resolves
# the joint: on a 12-face box the median edge is 500mm and the resulting
# 250mm slack admits almost anything. Coarse parts need refinement, not a
# smaller fraction.
_NEAR_CONTACT_EDGE_FRACTION = 0.5

# Endpoint gap below which two polylines from DIFFERENT meshes are stitched
# into one chain, as a multiple of the LOCAL sampling density of the two
# pieces being joined. Mesh-derived, not tuned: the two meshes do not share
# vertices, so a handoff gap of about one segment is expected.
#
# Overridable as 'stitch_gap_factor'. Must stay local — a value pooled over
# all pieces is dominated by whichever mesh is finer (a 1145-point arc against
# an 8-point chord gave a 1.4mm tolerance for a 14mm handoff) and then no real
# handoff is ever joined. Raise it if chains that clearly belong together stay
# separate; lower it if unrelated seams get spliced end to end.
_STITCH_GAP_FACTOR = 3.0

_MIN_LOOP_POINTS = 4

DEFAULT_EPSILON = 0.002


class ContactBoundaryExtractor:
    """Extract weld seams from the contact region with per-point ownership.

    Args:
        mesh_1: First mesh (world frame, watertight).
        mesh_2: Second mesh (world frame, watertight).
        params: Configuration dict. Reads 'epsilon', the two mesh-derived
            multipliers 'near_contact_edge_fraction' and 'stitch_gap_factor',
            and the PathCreator keys.

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
        self.epsilon = float(self.params.get('epsilon', DEFAULT_EPSILON))
        self.near_contact_fraction = float(self.params.get(
            'near_contact_edge_fraction', _NEAR_CONTACT_EDGE_FRACTION
        ))
        self.stitch_gap_factor = float(self.params.get(
            'stitch_gap_factor', _STITCH_GAP_FACTOR
        ))
        self.mesh = {1: mesh_1, 2: mesh_2}
        self._distance_cache: Dict[int, NDArray] = {}
        self._dihedral_cache: Dict[int, Dict[Tuple[int, int], float]] = {}
        self._sharp_tree_cache: Dict[int, Optional[KDTree]] = {}

    # ------------------------------------------------------------- routing

    def route(self) -> str:
        """Decide which extraction path this part pair needs.

        Returns:
            'contact' when the parts touch or are separated by a fit-up gap,
            'intersect' when they interpenetrate and require the exact
            corefinement intersection curve instead.
        """
        volume = float(
            (self._manifold(1) ^ self._manifold(2)).volume()
        )
        if volume > _INTERPENETRATION_VOLUME:
            logger.info(
                f'Parts interpenetrate ({volume * 1e9:.1f} mm^3 of overlap); '
                'the contact boundary is the buried rim, not the seam'
            )
            return 'intersect'
        return 'contact'

    def _manifold(self, side: int) -> manifold3d.Manifold:
        """Convert one mesh to a manifold3d solid for boolean tests."""
        mesh = self.mesh[side]
        data = manifold3d.Mesh(
            vert_properties=np.asarray(mesh.vertices, dtype=np.float32),
            tri_verts=np.asarray(mesh.faces, dtype=np.uint32),
        )
        return manifold3d.Manifold(data)

    # -------------------------------------------------------- contact mask

    def _surface_distance(
        self, mesh: trimesh.Trimesh, points: NDArray
    ) -> NDArray:
        """Exact distance from each point to `mesh`'s surface.

        Candidates come from two KD-trees, by face centroid and by vertex.
        The vertex tree is what makes this correct on fan-triangulated caps,
        where a containing triangle can be far from its own centroid; see
        _CLOSEST_CANDIDATES.
        """
        points = np.atleast_2d(points)
        faces = []

        count = min(_CLOSEST_CANDIDATES, len(mesh.faces))
        _, by_centroid = KDTree(mesh.triangles_center).query(points, k=count)
        faces.append(by_centroid.reshape(len(points), count))

        vcount = min(_CLOSEST_VERTICES, len(mesh.vertices))
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

    # --------------------------------------------------------------- loops

    def _loops(
        self, side: int, boundary: List[Tuple[int, int]]
    ) -> List[Tuple[List[int], bool]]:
        """Walk boundary edges into ordered chains of vertex indices.

        A boundary vertex normally has degree two, and the traversal closes
        into a loop. Degree can legitimately exceed two where two boundary
        curves cross — the arc/chord corner on a partly-overhanging part is
        one. Those vertices are treated as junctions: the boundary is cut
        there and emitted as open chains, since the continuation across a
        junction is genuinely ambiguous and guessing it would splice two
        different seams together.

        Returns:
            List of (vertex_indices, is_closed) pairs.
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

        for junction in junctions:
            for neighbour in adjacency[junction]:
                key = (min(junction, neighbour), max(junction, neighbour))
                if key not in used:
                    chains.append((walk(junction, neighbour), False))

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
            if len(chain) >= _MIN_LOOP_POINTS
        ]

    # ---------------------------------------------------------- side choice

    def _sharp_boundary(
        self, side: int, boundary: List[Tuple[int, int]]
    ) -> List[Tuple[int, int]]:
        """Contact-boundary edges that follow a real part edge of this mesh.

        Both meshes bound the same contact region, so both produce a boundary,
        and it is tempting to pick whichever one "is" the seam. That model is
        wrong. Where a part *terminates* on the other, its boundary runs along
        its own rim and those edges are sharp; where it merely *supports* the
        other, its boundary is an imprint cutting across coplanar faces. On a
        joint where one part overhangs the other, each mesh is the terminating
        one over part of the curve and the supporting one over the rest — the
        cylinder owns the arc, the plate owns the chord along its own edge.

        So neither mesh is "the" seam mesh, and no comparison between whole
        meshes selects correctly: measured on the test scene, sharp boundary
        length picks the plate at refine 4 and 8 and the cylinder only at
        refine 0. This returns each mesh's candidate contribution; which mesh
        actually owns any given POINT is settled later, per point, by _owner.

        Sharpness alone is not enough to be seam. A part's own edges away from
        the joint are perfectly sharp — a plate's outer rim most of all — so
        candidates must also sit on the other mesh (see _near_contact). Without
        that filter the plate emitted its entire 2400mm perimeter as weld.
        """
        table = self._edge_dihedrals(side)
        sharp = [
            edge for edge in boundary
            if table.get(edge, 0.0) > _COPLANAR_RAD
        ]
        if not sharp:
            return []

        # A sharp edge is only seam where the parts actually meet. The plate's
        # outer rim is sharp along its whole length and is not a weld.
        vertices = self.mesh[side].vertices
        index = np.asarray(sharp)
        midpoints = 0.5 * (vertices[index[:, 0]] + vertices[index[:, 1]])
        keep = self._near_contact(side, midpoints)
        dropped = int((~keep).sum())
        if dropped:
            logger.info(
                f'mesh_{side}: dropped {dropped} of {len(sharp)} sharp '
                'boundary edge(s) that are real part edges away from the joint'
            )
        return [edge for edge, ok in zip(sharp, keep) if ok]

    # ------------------------------------------------------------ epsilon

    def _validate_epsilon(self, side: int, epsilon: float) -> None:
        """Warn when epsilon is not on the plateau where the boundary is fixed.

        Rather than modelling where the safe band lies, this measures it: the
        extraction is repeated at neighbouring epsilon values and the boundary
        vertex set compared. On the plateau the set is identical.
        """
        _, reference = self._boundary_edges(side, epsilon)
        base = {v for edge in reference for v in edge}
        counts = [f'{epsilon * 1000:.2f}mm -> {len(base)}']
        stable = True

        for factor in _EPS_STABILITY_FACTORS:
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

    # ------------------------------------------------------------ assembly

    def _closest_faces(self, mesh: trimesh.Trimesh, points: NDArray) -> NDArray:
        """Index of the nearest face of `mesh` for each point."""
        points = np.atleast_2d(points)
        count = min(_CLOSEST_CANDIDATES, len(mesh.faces))
        _, by_centroid = KDTree(mesh.triangles_center).query(points, k=count)
        by_centroid = by_centroid.reshape(len(points), count)

        vcount = min(_CLOSEST_VERTICES, len(mesh.vertices))
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

    # ----------------------------------------------------- point ownership

    def _sharp_edge_tree(self, side: int) -> Optional[KDTree]:
        """KD-tree over points sampled along every sharp edge of one mesh.

        Ownership is decided by comparing distances to these trees, so the
        sampling has to be dense enough that a long sharp edge is not
        represented only by its endpoints.
        """
        if side in self._sharp_tree_cache:
            return self._sharp_tree_cache[side]

        mesh = self.mesh[side]
        table = mesh.face_adjacency_angles
        edges = mesh.face_adjacency_edges[table > _COPLANAR_RAD]
        if not len(edges):
            self._sharp_tree_cache[side] = None
            return None

        start, end = mesh.vertices[edges[:, 0]], mesh.vertices[edges[:, 1]]
        longest = float(np.linalg.norm(end - start, axis=1).max())
        steps = max(2, int(np.ceil(longest / max(self.epsilon, 1e-9))) + 1)
        steps = min(steps, 256)
        t = np.linspace(0.0, 1.0, steps)[:, None, None]
        samples = (start[None] * (1.0 - t) + end[None] * t).reshape(-1, 3)

        tree = KDTree(samples)
        self._sharp_tree_cache[side] = tree
        return tree

    def _owner(self, points: NDArray) -> Tuple[NDArray, NDArray]:
        """Which mesh carries the geometric edge at each point.

        This is the per-point form of SeamExtractor._classify_seed: a direct
        comparison between the two meshes, with no threshold deciding whether
        an edge is "sharp enough". At a seam one of the two parts terminates,
        so one of the two distances is essentially zero and the other is not.
        Ownership may change from point to point along a single chain, which
        is exactly what a joint with an overhang does.

        Returns:
            (owner, distance) — owner is 1 or 2 per point; distance is that
            point's distance to the winning mesh's nearest sharp edge.
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

    def _near_contact(self, side: int, points: NDArray) -> NDArray:
        """True where a point sits essentially on the OTHER mesh.

        A weld seam exists where the parts meet. A mesh's own part edges away
        from the joint are perfectly sharp and would otherwise be admitted:
        this is what let a plate emit its entire outer rim as seam.
        """
        other = self.mesh[2 if side == 1 else 1]
        slack = self.near_contact_fraction * self._median_edge(side)
        return self._surface_distance(other, points) <= self.epsilon + slack

    def _wall_normal(self, side: int, vertex: int, contact: NDArray) -> NDArray:
        """Area-weighted normal of the non-contact faces meeting a vertex.

        These are the faces on the wall rising away from the joint, so this is
        the seam's wall normal. It is selected by contact membership, not by
        an area-majority comparison, so it carries no label ambiguity.
        """
        mesh = self.mesh[side]
        incident = mesh.vertex_faces[vertex]
        incident = incident[incident >= 0]
        wall = incident[~contact[incident]]
        if not len(wall):
            wall = incident
        weights = mesh.area_faces[wall]
        normal = weights @ mesh.face_normals[wall]
        norm = np.linalg.norm(normal)
        return normal / norm if norm > 1e-12 else mesh.face_normals[wall[0]]

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

    # ---------------------------------------------------- cross-mesh stitch

    def _stitch(
        self, pieces: List[Tuple[NDArray, bool]]
    ) -> List[Tuple[NDArray, bool]]:
        """Join polylines from different meshes end to end into one chain.

        Where a joint overhangs, one part terminates over some of the seam and
        the other terminates over the rest, so each mesh contributes only its
        own portion. The two meshes share no vertices, so the pieces meet with
        a gap of roughly one segment; this closes those gaps and produces a
        single ordered chain whose ownership alternates along its length.

        Args:
            pieces: (positions, is_closed) polylines, any mesh.

        Returns:
            The same curve as a smaller number of longer polylines.
        """
        open_pieces = [list(p) for p, closed in pieces if not closed]
        result = [(p, True) for p, closed in pieces if closed]
        if len(open_pieces) < 2:
            return result + [(np.asarray(p), False) for p in open_pieces]

        def span(piece: List[NDArray]) -> float:
            """Local sampling density of one piece."""
            if len(piece) < 2:
                return 0.0
            return float(np.median(
                np.linalg.norm(np.diff(np.asarray(piece), axis=0), axis=1)
            ))

        # The two meshes can be sampled at wildly different densities (a
        # 1145-point arc against an 8-point chord), so the admissible gap is
        # taken from the two pieces being joined, never pooled across all of
        # them: a pooled median is dominated by whichever mesh is finer and
        # would refuse every real handoff.
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
                len(array) > _MIN_LOOP_POINTS
                and float(np.linalg.norm(array[0] - array[-1]))
                <= self.stitch_gap_factor * span(piece)
            )
            if closed:
                array = array[:-1]
            result.append((array, closed))
        return result

    def extract_chains(self) -> List[Tuple[List[SeamPoint], bool]]:
        """Build ordered SeamPoint chains with per-point edge ownership.

        Ownership is resolved per point, not per chain: at each point the two
        meshes are compared and whichever carries the geometric edge there
        owns it. A single chain may therefore alternate between meshes, which
        is what a partly overhanging joint actually does.

        Returns:
            List of (seam_points, is_closed) pairs. Empty when the parts do
            not touch, or when they interpenetrate and need the intersection
            path instead.
        """
        if self.route() == 'intersect':
            return []

        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]] = {}
        touching = False
        for side in (1, 2):
            contact, boundary = self._boundary_edges(side, self.epsilon)
            touching = touching or bool(boundary)
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

        # Pool both meshes' pieces, then stitch across meshes. Ownership is
        # NOT what selects a piece here; it is read back per point afterwards.
        pieces: List[Tuple[NDArray, bool]] = []
        for side in (1, 2):
            contact, sharp = marked[side]
            if not sharp:
                continue
            self._validate_epsilon(side, self.epsilon)
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
        owners = [p.refined_side for points, _ in chains for p in points]
        switches = sum(
            1 for points, _ in chains
            for a, b in zip(points, points[1:])
            if a.refined_side != b.refined_side
        )
        logger.info(
            f'{len(chains)} seam chain(s), {total} seam point(s); ownership '
            f'mesh_1={owners.count(1)} mesh_2={owners.count(2)}, '
            f'{switches} in-chain ownership change(s)'
        )
        return chains

    def _drop_coincident(
        self, pieces: List[Tuple[NDArray, bool]]
    ) -> List[Tuple[NDArray, bool]]:
        """Drop a piece that lies on top of a better-sampled one.

        On an edge-to-edge joint both parts terminate on the same curve, so
        both contribute it and it would be welded twice. Coincident pieces sit
        within epsilon of each other along their whole length — the same
        condition that defined contact, so no new tolerance is introduced.
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

    def _seam_points(
        self,
        positions: NDArray,
        marked: Dict[int, Tuple[NDArray, List[Tuple[int, int]]]],
    ) -> List[SeamPoint]:
        """Attach per-point ownership and normals to an ordered polyline."""
        owner, _ = self._owner(positions)

        # The edge-carrying mesh supplies the wall normal; the other supplies
        # the base surface. Both are read off the owner, so neither comes from
        # an area-majority comparison that can tie and flip.
        normal_main = np.zeros((len(positions), 3))
        normal_secondary = np.zeros((len(positions), 3))
        for side in (1, 2):
            rows = np.nonzero(owner == side)[0]
            if not len(rows):
                continue
            other = self.mesh[2 if side == 1 else 1]
            base = self._closest_faces(other, positions[rows])
            normal_main[rows] = other.face_normals[base]

            # Wall normal from the faces NOT in contact, so it points along
            # the surface rising out of the joint rather than into it.
            contact = marked[side][0]
            _, nearest = KDTree(self.mesh[side].vertices).query(positions[rows])
            normal_secondary[rows] = [
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
                normal_main=normal_main[i],
                normal_secondary=normal_secondary[i],
                on_edge_1=bool(on_edge[1][i]),
                on_edge_2=bool(on_edge[2][i]),
                refined_side=int(owner[i]),
            )
            for i in range(len(positions))
        ]

    def extract_seams(self) -> List[Seam]:
        """Extract classified Seam objects, mirroring SeamExtractor's output."""
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
