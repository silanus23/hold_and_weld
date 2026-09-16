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

"""MeshFields, cached geometric queries over the two meshes being joined.

Answers questions about the meshes alone, with no notion of seams, chains or ownership. Split
out of SeamExtractorMesh so it can request geometry rather than caching it itself.
"""

import logging
from typing import Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from .params import SeamExtractorMeshParams

logger = logging.getLogger(__name__)


class MeshFields:
    """Cached geometric queries over the two meshes being joined."""

    def __init__(self, mesh: Dict[int, trimesh.Trimesh], cfg: SeamExtractorMeshParams) -> None:
        """Initialize the field cache from the two meshes and tuning params."""
        self.mesh = mesh
        self.cfg = cfg

        self._distance_cache: Dict[int, NDArray] = {}
        self._dihedral_cache: Dict[int, Dict[Tuple[int, int], float]] = {}
        self._sharp_edge_cache: Dict[int, Optional[Tuple[NDArray, NDArray]]] = {}
        self._sharp_tree_cache: Dict[int, Optional[KDTree]] = {}
        self._centroid_tree_cache: Dict[int, KDTree] = {}
        self._vertex_tree_cache: Dict[int, KDTree] = {}
        self._edge_scale_cache: Dict[int, NDArray] = {}
        self._median_edge_cache: Dict[int, float] = {}
        self._turning_cache: Dict[int, Tuple[KDTree, NDArray]] = {}

    def centroid_tree(self, side: int) -> KDTree:
        """KD-tree over one mesh's triangle centroids, built once per side."""
        if side in self._centroid_tree_cache:
            return self._centroid_tree_cache[side]

        tree = KDTree(self.mesh[side].triangles_center)
        self._centroid_tree_cache[side] = tree
        return tree

    def vertex_tree(self, side: int) -> KDTree:
        """KD-tree over one mesh's vertices, built once per side."""
        if side in self._vertex_tree_cache:
            return self._vertex_tree_cache[side]

        tree = KDTree(self.mesh[side].vertices)
        self._vertex_tree_cache[side] = tree
        return tree

    def _face_candidates(self, side: int, points: NDArray) -> NDArray:
        """Faces worth testing for the nearest surface point, per point.

        Nearest centroids plus faces incident on the nearest vertices. A centroid can be far on a
        long thin triangle, and a vertex fan catches those.
        """
        mesh = self.mesh[side]
        n_pts = len(points)
        count = min(self.cfg.closest_face_candidates, len(mesh.faces))
        _, by_centroid = self.centroid_tree(side).query(points, k=count)
        by_centroid = by_centroid.reshape(n_pts, count)

        vcount = min(self.cfg.closest_vertex_candidates, len(mesh.vertices))
        _, near_vertices = self.vertex_tree(side).query(points, k=vcount)
        incident = mesh.vertex_faces[near_vertices.reshape(n_pts, vcount)]

        # vertex_faces pads short rows with -1; swap those for an already valid candidate (nearest
        # centroid) instead of an invalid face index. Flattened to an explicit width, not -1: at
        # n_pts == 0 the target has zero elements and -1 can't be inferred from that.
        incident = np.where(
            incident < 0, by_centroid[:, :1, None], incident
        ).reshape(n_pts, vcount * mesh.vertex_faces.shape[1])
        return np.hstack([by_centroid, incident])

    def _nearest(self, side: int, points: NDArray) -> Tuple[NDArray, NDArray]:
        """Distance to side's surface and the face carrying it, per point."""
        mesh = self.mesh[side]
        candidates = self._face_candidates(side, points)
        best_faces = np.zeros(len(points), dtype=np.int64)
        best_distances = np.full(len(points), np.inf)

        # Strict '<' below: a tie between distinct faces resolves to whichever column is tested
        # first, not any geometric rule.
        for col in range(candidates.shape[1]):
            faces = candidates[:, col]
            closest = trimesh.triangles.closest_point(mesh.triangles[faces], points)
            distances = np.linalg.norm(closest - points, axis=1)
            closer_mask = distances < best_distances
            best_distances[closer_mask] = distances[closer_mask]
            best_faces[closer_mask] = faces[closer_mask]
        return best_distances, best_faces

    def surface_distance(self, side: int, points: NDArray) -> NDArray:
        """Exact distance from each point to mesh side's surface in meters."""
        return self._nearest(side, points)[0]

    def closest_faces(self, side: int, points: NDArray) -> NDArray:
        """Index of the nearest face of mesh side for each point."""
        return self._nearest(side, points)[1]

    def vertex_edge_scale(self, side: int) -> NDArray:
        """Local surface scale at each vertex of one mesh.

        The characteristic length used to size the coverage-kernel radius around a point (see
        extract_seams): a value too small relative to the surrounding triangle size makes the
        integral read 0 or overshoot on luck, so this has to track actual local geometry rather
        than being a fixed constant.

        Computed as the mean length of edges incident to each vertex, not the median: at an
        irregular vertex most incident edges are short and one or two are long, bridging into
        coarser geometry nearby. The median reports the dense short cluster and discards the long
        edges, understating the true local scale; the mean stays sensitive to them.

        Returns:
            One scale value per vertex of that mesh, in mesh length units.
        """
        if side in self._edge_scale_cache:
            return self._edge_scale_cache[side]

        mesh = self.mesh[side]
        edges = mesh.edges_unique
        lengths = np.linalg.norm(mesh.vertices[edges[:, 0]] - mesh.vertices[edges[:, 1]], axis=1)

        total = np.zeros(len(mesh.vertices))
        count = np.zeros(len(mesh.vertices))
        for column in (0, 1):
            np.add.at(total, edges[:, column], lengths)
            np.add.at(count, edges[:, column], 1.0)

        scale = total / np.maximum(count, 1.0)
        self._edge_scale_cache[side] = scale
        return scale

    def median_edge(self, side: int) -> float:
        """Median edge length of one mesh, computed once per side."""
        if side in self._median_edge_cache:
            return self._median_edge_cache[side]

        mesh = self.mesh[side]
        lengths = np.linalg.norm(
            mesh.vertices[mesh.edges_unique[:, 0]]
            - mesh.vertices[mesh.edges_unique[:, 1]], axis=1)
        median = float(np.median(lengths))
        self._median_edge_cache[side] = median
        return median

    def face_distance(self, side: int) -> NDArray:
        """Distance from each face centroid of side to the OTHER surface.

        The contact field: `<= epsilon` is what marks a face as touching.
        """
        if side in self._distance_cache:
            return self._distance_cache[side]
        distance = self.surface_distance(3 - side, self.mesh[side].triangles_center)
        self._distance_cache[side] = distance
        return distance

    def contact_boundary(self, side: int, epsilon: float) -> Tuple[NDArray, List[Tuple[int, int]]]:
        """Contact mask at `epsilon` and the edges with exactly one contact face.

        Edges come back in order of first appearance across the contact faces, which `loops` walks
        in.
        """
        mesh = self.mesh[side]
        contact = self.face_distance(side) <= epsilon
        stream = mesh.faces_unique_edges[contact].ravel()
        unique, first, counts = np.unique(stream, return_index=True, return_counts=True)
        once = counts == 1
        edges = mesh.edges_unique[unique[once][np.argsort(first[once])]]
        return contact, [(int(a), int(b)) for a, b in edges]

    def fan_normal(self, side: int, vertex: int, faces: NDArray) -> NDArray:
        """Area-weighted normal of the selected faces meeting a vertex.

        Falls back to the whole fan when none of it is selected, then to the vertex normal when
        the fan is degenerate.

        Args:
            faces: Boolean mask over this mesh's faces.

        Returns:
            Unit normal, or zeros when nothing usable is left.
        """
        mesh = self.mesh[side]
        incident = mesh.vertex_faces[vertex]
        incident = incident[incident >= 0]
        chosen = incident[faces[incident]]
        if not len(chosen):
            chosen = incident

        # A degenerate face has zero area and a zero normal in trimesh, so an all-degenerate fan
        # weights to zero rather than a direction.
        for normal in (mesh.area_faces[chosen] @ mesh.face_normals[chosen],
                       mesh.vertex_normals[vertex]):
            norm = np.linalg.norm(normal)
            if norm > 1e-12:
                return normal / norm
        return np.zeros(3)

    def edge_dihedrals(self, side: int) -> Dict[Tuple[int, int], float]:
        """Map each interior mesh edge to the angle between its two triangle faces.

            theta_e = arccos(normal_a . normal_b)

        Unsigned angle between the two faces' unit normals, not the classical interior dihedral
        (which is pi - theta_e on a convex edge). This is the turning angle: 0 for coplanar faces,
        growing with how sharply the surface bends there, independent of convex/concave sign, the
        quantity `turning_density` sums.

        `face_adjacency_edges`/`_angles` come from trimesh already paired and already restricted to
        edges with two faces. A boundary edge (one face only, no second normal to compare against)
        never appears here. This file never walks triangles to find that itself; every other user
        of these two arrays in this file inherits the same exclusion.
        """
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

    def _sharp_edges(self, side: int) -> Optional[Tuple[NDArray, NDArray]]:
        """Endpoints of every sharp edge of one mesh, or None if it has none."""
        if side in self._sharp_edge_cache:
            return self._sharp_edge_cache[side]

        mesh = self.mesh[side]
        edges = mesh.face_adjacency_edges[mesh.face_adjacency_angles > self.cfg.edge_angle_min]
        pair = (
            None if not len(edges)
            else (mesh.vertices[edges[:, 0]], mesh.vertices[edges[:, 1]])
        )
        self._sharp_edge_cache[side] = pair
        return pair

    def _sharp_midpoint_tree(self, side: int) -> Optional[KDTree]:
        """KD-tree over the midpoints of one mesh's sharp edges."""
        if side in self._sharp_tree_cache:
            return self._sharp_tree_cache[side]

        pair = self._sharp_edges(side)
        tree = None if pair is None else KDTree(0.5 * (pair[0] + pair[1]))
        self._sharp_tree_cache[side] = tree
        return tree

    def sharp_edge_distance(self, side: int, points: NDArray) -> NDArray:
        """Exact distance from each point to the nearest sharp edge of a mesh.

        Point-to-SEGMENT, over the edges whose midpoints are nearest. Replaced a sampled-point
        KD-tree whose quantization error could flip ownership at a corner once points are refined
        off the lattice.

        Returns:
            All `inf` when this mesh has no sharp edge at all.
        """
        pair = self._sharp_edges(side)
        if pair is None:
            return np.full(len(points), np.inf)

        start, end = pair
        count = min(self.cfg.sharp_edge_candidates, len(start))
        _, candidates = self._sharp_midpoint_tree(side).query(points, k=count)
        candidates = candidates.reshape(len(points), count)

        segment = end - start
        # A degenerate edge collapses to its start, which the clip below then returns as the
        # closest point; the guard only keeps the division safe.
        span = np.einsum('ij,ij->i', segment, segment)
        span = np.where(span > 0.0, span, 1e-30)

        distance = np.full(len(points), np.inf)
        for column in range(count):
            index = candidates[:, column]
            offset = points - start[index]
            t = np.clip(np.einsum('ij,ij->i', offset, segment[index]) / span[index], 0.0, 1.0)
            closest = start[index] + t[:, None] * segment[index]
            distance = np.minimum(distance, np.linalg.norm(closest - points, axis=1))
        return distance

    def _turning_field(self, side: int) -> Tuple[KDTree, NDArray]:
        """Midpoint tree and turning weight over ALL adjacency edges.

        The weight is `dihedral * shared_edge_length`, whose sum over a region is the discrete
        total curvature of that region.
        """
        if side in self._turning_cache:
            return self._turning_cache[side]

        mesh = self.mesh[side]
        edges = mesh.face_adjacency_edges
        start = mesh.vertices[edges[:, 0]]
        end = mesh.vertices[edges[:, 1]]
        weight = mesh.face_adjacency_angles * np.linalg.norm(end - start, axis=1)
        field = (KDTree(0.5 * (start + end)), weight)
        self._turning_cache[side] = field
        return field

    def turning_density(self, side: int, points: NDArray, radius: float) -> NDArray:
        """Measure turning per unit area near each point, on one mesh alone.

            density = sum(dihedral * edge_length) / sum(face area)

        Separates a real part edge from a tessellated curve where a raw angle threshold cannot:
        facet seams and a real rim both read as sharp, but density doesn't. Flat reads 0, a
        cylinder of radius r reads 1/r at any tessellation.

        Summed as a ratio of SUMS, not per-edge, so it stays stable under remeshing instead of
        inflating as facet seams get refined.

        Args:
            radius: See `SeamExtractorMesh._ownership_radius` for the band this has to sit in.

        Returns:
            Zero where no face centroid falls inside the ball.
        """
        n_pts = len(points)
        mesh = self.mesh[side]
        tree, weight = self._turning_field(side)

        turning = np.fromiter(
            (float(weight[hits].sum()) if hits else 0.0
             for hits in tree.query_ball_point(points, radius)),
            dtype=np.float64, count=n_pts,
        )
        area = np.fromiter(
            (float(mesh.area_faces[hits].sum()) if hits else 0.0
             for hits in self.centroid_tree(side).query_ball_point(
                 points, radius)),
            dtype=np.float64, count=n_pts,
        )
        return np.where(area > 0.0, turning / np.where(area > 0.0, area, 1.0), 0.0)

    def coverage(self, side: int, point: NDArray, rho: float, mating: NDArray) -> float:
        """Facing-weighted fraction of the other mesh's surface around a point.

        Reads 1 where the other mesh's mating surface fills the neighbourhood, 0 where it is
        absent, 1/2 on the boundary.

        Note:
            The `mating`-facing weight is what makes the boundary exist at all: the other part's
            WALL stands ON the joint, so unweighted it would fill the neighbourhood on BOTH sides
            and the measure would read ~1 everywhere. The integral is also sampled per triangle, so
            one straddling the boundary is counted whole or not at all.

        Args:
            side: The mesh that OWNS the point; the integral runs over the other one.
            rho: Kernel radius. Must span the triangles of the mesh being integrated, not only the
                owner's.
            mating: Unit normal of the owner's own contact surface there.

        Returns:
            Zero when rho catches no triangle.
        """
        if not rho > 0.0:
            return 0.0
        other_side = 3 - side
        other = self.mesh[other_side]
        index = self.centroid_tree(other_side).query_ball_point(point, rho)
        if not len(index):
            return 0.0
        index = np.asarray(index, dtype=np.int64)
        t = np.linalg.norm(other.triangles_center[index] - point, axis=1) / rho
        facing = np.clip(-(other.face_normals[index] @ mating), 0.0, None)
        # (1 - t^2)^3: a smooth, compactly-supported kernel (1 at t=0, 0 at t=1, zero slope at both
        # ends) rather than a hard radius cutoff.
        weight = (1.0 - t ** 2) ** 3 * other.area_faces[index] * facing
        return float((weight / (np.pi * rho ** 2 / 4.0)).sum())

    def slide_to_boundary(
        self,
        position: NDArray,
        direction: NDArray,
        side: int,
        rho: float,
        mating: NDArray,
    ) -> Optional[NDArray]:
        """Slide a point along `direction` onto the half level set of coverage.

        Searches BOTH ways, since the run end isn't guaranteed to start inside, it can sit just
        past the boundary and still belong on the level set, only backwards.

        Args:
            direction: Usually the chain tangent at a run's outer end.
            rho: Kernel radius, also the span searched each way.
            mating: Unit normal of the owner's own contact surface there.

        Returns:
            None when neither span brackets the crossing, too coarse to resolve a boundary, or not
            near one. The caller then leaves the point on its vertex.
        """
        if not rho > 0.0:
            return None

        here = self.coverage(side, position, rho, mating)
        ahead = self.coverage(side, position + rho * direction, rho, mating)
        if here >= 0.5 > ahead:
            within, beyond = 0.0, rho
        else:
            behind = self.coverage(side, position - rho * direction, rho, mating)
            if behind >= 0.5 > here:
                within, beyond = -rho, 0.0
            else:
                return None

        # Invariant: coverage at `within` is >= 1/2 and at `beyond` is < 1/2.
        for _ in range(self.cfg.coverage_bisection_steps):
            middle = 0.5 * (within + beyond)
            if self.coverage(side, position + middle * direction, rho, mating) >= 0.5:
                within = middle
            else:
                beyond = middle
        return position + 0.5 * (within + beyond) * direction


def reject_holes(
    inside: NDArray, positions: NDArray, rho: NDArray,
    is_closed: bool = False,
) -> NDArray:
    """Restore interior drop blocks that are holes rather than corners.

    A block at a chain END is an ordinary trim. A block in the MIDDLE splits the chain, and its
    ends get slid inward onto the level set, right at a real crossing, but catastrophic anywhere
    else, since it carves a stretch out of the middle of a weld.

    At a real crossing the seam leaves and re-enters the contact at the SAME place, so the
    surviving points either side sit almost on top of each other. A coverage failure mid-seam
    leaves a hole instead, whose ends are as far apart as the stretch that was lost, so farther
    apart than the analysis scale means keep those points rather than trust a field that's
    evidently gone wrong there.

    A CLOSED chain has no end for a block to sit at, so a block touching both ends of the array is
    one block straddling the wrap point.

    Args:
        inside: Per-point contact mask, as thresholding `coverage` at a half produced it. Modified
            in place.
        rho: Kernel radius used at each point; its maximum over a block is that block's analysis
            scale.
        is_closed: True when the chain wraps, so index 0 follows index N-1 and no block sits at an
            end.

    Returns:
        The same mask, with hole blocks restored to True.
    """
    count = len(inside)
    blocks: List[Tuple[int, int]] = []
    start: Optional[int] = None
    for i, ok in enumerate(inside):
        if not ok and start is None:
            start = i
        elif ok and start is not None:
            blocks.append((start, i - 1))
            start = None
    if start is not None:
        blocks.append((start, count - 1))

    # On a closed chain, a block against each end is one block straddling the wrap, joined here so
    # it's measured once, not as two phantom ends.
    if is_closed and len(blocks) > 1 and blocks[0][0] == 0 and blocks[-1][1] == count - 1:
        head, tail = blocks.pop(0), blocks.pop()
        blocks.append((tail[0], head[1] + count))

    restored = 0
    for first, last in blocks:
        if not is_closed and (first == 0 or last == count - 1):
            continue                      # a chain end, so an ordinary trim
        # Modulo, so a block written across the wrap reads its neighbours and its own points from
        # the far end of the array.
        block = [i % count for i in range(first, last + 1)]
        before, after = (first - 1) % count, (last + 1) % count
        span = float(np.linalg.norm(positions[after] - positions[before]))
        scale = float(np.max(rho[block]))
        if span <= scale:
            continue                      # ends meet: a crossing
        inside[block] = True
        restored += len(block)
        logger.warning(
            f'Coverage drops {len(block)} point(s) mid-chain whose '
            f'surviving ends are {span * 1000:.2f}mm apart, past the '
            f'{scale * 1000:.2f}mm analysis scale; that is a hole, not a '
            'corner, so the points are kept. Suspect rho or the mating '
            'normal there.'
        )

    if restored:
        logger.info(f'Restored {restored} point(s) over {len(blocks)} drop block(s)')
    return inside
