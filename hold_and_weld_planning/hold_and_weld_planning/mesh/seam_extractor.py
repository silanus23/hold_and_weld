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

"""Extract and classify weld seam points from a pair of touching mesh shells."""

from collections import deque
from dataclasses import dataclass
import logging
from typing import Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from ..core.seam import Seam

try:
    from . import mesh_intersection as _mesh_intersection
except ImportError as e:
    raise ImportError(
        'mesh_intersection C++ module not found. '
        'Build it with: cd mesh_intersection && mkdir build && cd build && cmake .. && make'
    ) from e

logger = logging.getLogger(__name__)


@dataclass
class SeamPoint:
    """Single point on a weld seam with surface normal information.

    Attributes:
        position:          Refined 3D position on the geometric edge (3,).
        normal_main:       Surface normal of the base plate (3,).
        normal_secondary:  Surface normal of the secondary part (3,).
        on_edge_1:         True if the mesh_1-side point lies on a geometric
                            edge (bimodal local normals).
        on_edge_2:         True if the mesh_2-side point lies on a geometric
                            edge (bimodal local normals).
        refined_side:      Which mesh this point was refined against: 1 for
                            mesh_1, 2 for mesh_2.
    """

    position: NDArray
    normal_main: NDArray
    normal_secondary: NDArray
    on_edge_1: bool
    on_edge_2: bool
    refined_side: int


class SeamExtractor:
    """Extract weld seam points from two touching mesh shells.

    Calls C++ get_seam_vertices, picks per-point which mesh side is the true
    edge side (comparative normal-covariance bimodality), refines the chosen
    side's position via a local field scan, and delegates to PathCreator.
    """

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
            params: Configuration dict with keys: epsilon, covariance_radius,
                    edge_threshold, pairing_radius, num_smooth_points,
                    scan_point_spacing.

        Raises:
            ValueError: If either mesh is not watertight, or if pairing_radius
                        is smaller than epsilon.
        """
        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')

        self.mesh_1 = mesh_1
        self.mesh_2 = mesh_2
        self.params = params

        self.epsilon = params.get('epsilon', 1e-3)
        self.covariance_radius = params.get('covariance_radius', 0.05)
        self.edge_threshold = params.get('edge_threshold', 0.3)
        self.pairing_radius = params.get('pairing_radius', 0.01)
        self.num_smooth_points = params.get('num_smooth_points', 100)
        self.scan_point_spacing = params.get('scan_point_spacing', 0.004)

        if self.pairing_radius < self.epsilon:
            raise ValueError(
                f'pairing_radius ({self.pairing_radius}) must be >= '
                f'epsilon ({self.epsilon}), otherwise a face detected as '
                'in-contact by epsilon can fail to find a pairing partner.'
            )

        self.chain_1: Optional[Dict[str, NDArray]] = None
        self.chain_2: Optional[Dict[str, NDArray]] = None
        self.pairs: Dict[int, int] = {}

        self._seam_1_slices: List[Tuple[int, int, bool]] = []

        self._kdtree_chain1: Optional[KDTree] = None
        self._kdtree_chain2: Optional[KDTree] = None

        self._mesh_1_data = self._prepare_mesh_data(mesh_1)
        self._mesh_2_data = self._prepare_mesh_data(mesh_2)

    def _prepare_mesh_data(self, mesh: trimesh.Trimesh) -> Dict:
        """Precompute per-mesh lookup structures used by field-scan refinement."""
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        face_normals = np.asarray(mesh.face_normals, dtype=np.float64)
        area_faces = np.asarray(mesh.area_faces, dtype=np.float64)
        face_centroids = np.asarray(mesh.triangles_center, dtype=np.float64)

        return {
            'mesh': mesh,
            'vertices': vertices,
            'vertex_kdtree': KDTree(vertices),
            'face_normals': face_normals,
            'area_faces': area_faces,
            'face_centroids': face_centroids,
            'face_centroid_kdtree': KDTree(face_centroids),
            'vertex_faces': mesh.vertex_faces,
            'face_adjacency': self._build_face_adjacency(mesh),
        }

    def _build_face_adjacency(self, mesh: trimesh.Trimesh) -> Dict[int, List[int]]:
        """Build a face-index adjacency list from trimesh's face_adjacency pairs."""
        adjacency: Dict[int, List[int]] = {}
        for f1, f2 in mesh.face_adjacency:
            adjacency.setdefault(int(f1), []).append(int(f2))
            adjacency.setdefault(int(f2), []).append(int(f1))
        return adjacency

    def _get_local_normals(
        self,
        point: NDArray,
        tree: KDTree,
        normals: NDArray,
    ) -> NDArray:
        """Return normals within covariance_radius of point from tree/normals."""
        idxs = tree.query_ball_point(point, self.covariance_radius)
        if len(idxs) == 0:
            return np.empty((0, 3))
        return normals[np.array(idxs)]

    def _compute_local_covariance(
        self,
        point: NDArray,
        tree: KDTree,
        normals: NDArray,
    ) -> Tuple[Optional[NDArray], Optional[NDArray]]:
        """Return eigendecomposition of normal covariance within covariance_radius, or (None, None)."""
        local_normals = self._get_local_normals(point, tree, normals)

        if len(local_normals) < 3:
            return None, None

        cov = np.cov(local_normals.T)  # (3, 3)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)

        return eigenvalues, eigenvectors

    def _is_edge_contact(self, eigenvalues: NDArray) -> bool:
        """Return True if σ = λ2/sum(λ) >= edge_threshold (bimodal normals at an edge)."""
        total = np.sum(eigenvalues)
        if total < 1e-10:
            return False
        sigma = eigenvalues[2] / total
        return bool(sigma >= self.edge_threshold)

    def _compute_sigma_and_edge(
        self,
        point: NDArray,
        tree: KDTree,
        normals: NDArray,
    ) -> Tuple[float, bool]:
        """Return (sigma, is_edge) for point's local normal covariance against tree/normals."""
        eigenvalues, _ = self._compute_local_covariance(point, tree, normals)
        if eigenvalues is None:
            return 0.0, False

        total = np.sum(eigenvalues)
        if total < 1e-10:
            return 0.0, False

        sigma = float(eigenvalues[2] / total)
        return sigma, self._is_edge_contact(eigenvalues)

    def _load_ordered_chains(
        self,
        seams_1: List[Dict],
        seams_2: List[Dict],
    ) -> None:
        """Concatenate mesh_1 seams into chain_1 (recording index slices) and pool mesh_2 into chain_2."""
        pos_1_parts: List[NDArray] = []
        nrm_1_parts: List[NDArray] = []
        self._seam_1_slices = []
        cursor = 0

        for s in seams_1:
            v = np.asarray(s['vertices'], dtype=np.float64)
            nrm = np.asarray(s['normals'], dtype=np.float64)
            if len(v) < 2:
                continue
            pos_1_parts.append(v)
            nrm_1_parts.append(nrm)
            self._seam_1_slices.append((cursor, cursor + len(v), bool(s['is_closed'])))
            cursor += len(v)

        pos_2_parts: List[NDArray] = []
        nrm_2_parts: List[NDArray] = []

        for s in seams_2:
            v = np.asarray(s['vertices'], dtype=np.float64)
            nrm = np.asarray(s['normals'], dtype=np.float64)
            if len(v) < 1:
                continue
            pos_2_parts.append(v)
            nrm_2_parts.append(nrm)

        if not pos_1_parts or not pos_2_parts:
            self.chain_1 = {'positions': np.empty((0, 3)), 'normals': np.empty((0, 3))}
            self.chain_2 = {'positions': np.empty((0, 3)), 'normals': np.empty((0, 3))}
            self._seam_1_slices = []
            return

        self.chain_1 = {
            'positions': np.vstack(pos_1_parts),
            'normals': np.vstack(nrm_1_parts),
        }
        self.chain_2 = {
            'positions': np.vstack(pos_2_parts),
            'normals': np.vstack(nrm_2_parts),
        }

        logger.debug(
            f'Loaded ordered chains: chain_1={len(self.chain_1["positions"])} points '
            f'in {len(self._seam_1_slices)} seam(s), '
            f'chain_2={len(self.chain_2["positions"])} points'
        )

    def _pair_chains(self) -> None:
        """Pair chain_1 and chain_2 points by nearest-neighbour proximity within pairing_radius."""
        self._kdtree_chain2 = KDTree(self.chain_2['positions'])
        self.pairs = {}

        for i, p in enumerate(self.chain_1['positions']):
            dist, j = self._kdtree_chain2.query(p)
            if dist <= self.pairing_radius:
                self.pairs[i] = int(j)

        logger.debug(
            f'Paired {len(self.pairs)}/{len(self.chain_1["positions"])} chain_1 point(s)'
        )

    def _collect_nearby_faces(
        self,
        mesh_data: Dict,
        anchor_position: NDArray,
        anchor_vertex_idx: int,
    ) -> set:
        """Flood-fill face adjacency from anchor_vertex_idx, bounded by covariance_radius.

        Stays on mesh topology (no tangent-plane assumption needed) to gather
        the set of faces to sample candidate points from.
        """
        adjacency = mesh_data['face_adjacency']
        face_centroids = mesh_data['face_centroids']
        vertex_faces_row = mesh_data['vertex_faces'][anchor_vertex_idx]
        seed_faces = [int(f) for f in vertex_faces_row if f != -1]

        visited: set = set()
        frontier: deque = deque()

        for f in seed_faces:
            d = np.linalg.norm(face_centroids[f] - anchor_position)
            if d <= self.covariance_radius:
                visited.add(f)
                frontier.append(f)

        while frontier:
            f = frontier.popleft()
            for nb in adjacency.get(f, []):
                if nb in visited:
                    continue
                d = np.linalg.norm(face_centroids[nb] - anchor_position)
                if d <= self.covariance_radius:
                    visited.add(nb)
                    frontier.append(nb)

        if not visited:
            # Fallback: anchor's own incident faces, regardless of the radius
            # check, so refinement always has at least a seed neighborhood.
            visited = set(seed_faces)

        return visited

    def _field_scan_refine(
        self,
        mesh_data: Dict,
        anchor_position: NDArray,
        anchor_vertex_idx: int,
    ) -> NDArray:
        """Sample candidate points on the mesh near the anchor and return the one
        with highest normal-covariance bimodality (σ), i.e. closest to the true edge.
        """
        face_idx_set = self._collect_nearby_faces(mesh_data, anchor_position, anchor_vertex_idx)

        candidates = [anchor_position]

        if face_idx_set:
            face_indices = np.array(sorted(face_idx_set))
            total_area = float(np.sum(mesh_data['area_faces'][face_indices]))
            n_samples = max(8, int(total_area / (self.scan_point_spacing ** 2)))

            weights = np.zeros(len(mesh_data['face_normals']))
            weights[face_indices] = mesh_data['area_faces'][face_indices]

            try:
                sampled_points, _ = trimesh.sample.sample_surface(
                    mesh_data['mesh'], n_samples, face_weight=weights
                )
                candidates.extend(list(sampled_points))
            except Exception as e:
                logger.debug(f'Field-scan sampling failed, using anchor only: {e}')

        tree = mesh_data['face_centroid_kdtree']
        normals = mesh_data['face_normals']

        best_sigma = -np.inf
        best_position = anchor_position.copy()

        for cand in candidates:
            cand = np.asarray(cand, dtype=np.float64)
            sigma, _ = self._compute_sigma_and_edge(cand, tree, normals)
            if sigma > best_sigma:
                best_sigma = sigma
                best_position = cand.copy()

        return best_position

    def _find_side_switch_anchor(
        self,
        mesh_data: Dict,
        last_position: NDArray,
        direction: Optional[NDArray],
    ) -> Tuple[NDArray, int]:
        """Find the anchor vertex on the new mesh side when switching sides mid-seam.

        Picks the nearest vertex to last_position, but if a rough travel
        direction is known, restricts candidates to ones ahead of
        last_position along that direction (to avoid walking backward).
        """
        vertices = mesh_data['vertices']
        k = min(10, len(vertices))
        dists, idxs = mesh_data['vertex_kdtree'].query(last_position, k=k)
        dists = np.atleast_1d(dists)
        idxs = np.atleast_1d(idxs)

        chosen_idx = int(idxs[0])

        if direction is not None:
            dir_norm = np.linalg.norm(direction)
            if dir_norm > 1e-10:
                dir_unit = direction / dir_norm
                forward = [
                    (d, int(i)) for d, i in zip(dists, idxs)
                    if np.dot(vertices[int(i)] - last_position, dir_unit) > 0
                ]
                if forward:
                    forward.sort(key=lambda x: x[0])
                    chosen_idx = forward[0][1]

        return vertices[chosen_idx], chosen_idx

    def _build_seam_point(
        self,
        i: int,
        j: int,
        refined_position: NDArray,
        side: int,
        edge_1: bool,
        edge_2: bool,
    ) -> SeamPoint:
        """Assemble a SeamPoint from chain indices, the refined position, and chosen side.

        The refinement side carries the geometric edge, so its normal is the
        secondary part's normal; the other side's normal is the main (base) one.
        """
        normal_1 = self.chain_1['normals'][i]
        normal_2 = self.chain_2['normals'][j]

        if side == 1:
            normal_main, normal_secondary = normal_2, normal_1
        else:
            normal_main, normal_secondary = normal_1, normal_2

        return SeamPoint(
            position=refined_position,
            normal_main=normal_main,
            normal_secondary=normal_secondary,
            on_edge_1=edge_1,
            on_edge_2=edge_2,
            refined_side=side,
        )

    def extract_seams(self) -> List[Seam]:
        """Extract weld seams from the mesh pair and return Seam objects for WeldPlanner."""
        from .path_creator import PathCreator

        verts1 = np.asarray(self.mesh_1.vertices, dtype=np.float64)
        faces1 = np.asarray(self.mesh_1.faces, dtype=np.int32)
        verts2 = np.asarray(self.mesh_2.vertices, dtype=np.float64)
        faces2 = np.asarray(self.mesh_2.faces, dtype=np.int32)

        logger.info('Calling C++ seam vertex extraction...')

        try:
            seams_1 = _mesh_intersection.get_seam_vertices(
                verts1, faces1, verts2, faces2, self.epsilon
            )
            seams_2 = _mesh_intersection.get_seam_vertices(
                verts2, faces2, verts1, faces1, self.epsilon
            )
        except RuntimeError as e:
            logger.error(f'C++ extraction failed: {e}')
            return []

        if len(seams_1) == 0 or len(seams_2) == 0:
            logger.warning(
                f'No seams from C++: mesh_1={len(seams_1)}, mesh_2={len(seams_2)}'
            )
            return []

        logger.info(
            f'C++ returned {len(seams_1)} seam(s) from mesh_1, '
            f'{len(seams_2)} from mesh_2'
        )

        self._load_ordered_chains(seams_1, seams_2)

        if len(self.chain_1['positions']) == 0 or len(self.chain_2['positions']) == 0:
            logger.warning('No usable chain points after loading ordered seams')
            return []

        self._kdtree_chain1 = KDTree(self.chain_1['positions'])

        self._pair_chains()

        if not self.pairs:
            logger.warning(
                f'No pairs found within pairing_radius={self.pairing_radius}m'
            )
            return []

        logger.info(
            f'Refining and classifying {len(self.pairs)} seam point(s) '
            f'across {len(self._seam_1_slices)} seam(s)...'
        )

        path_creator = PathCreator(self.params)
        seams: List[Seam] = []
        failed = 0
        discarded = 0

        for start, end, _is_closed in self._seam_1_slices:
            seam_points: List[SeamPoint] = []

            prev_side: Optional[int] = None
            prev_refined_positions: List[NDArray] = []

            for i in range(start, end):
                j = self.pairs.get(i)
                if j is None:
                    continue

                try:
                    sigma_1, edge_1 = self._compute_sigma_and_edge(
                        self.chain_1['positions'][i], self._kdtree_chain1, self.chain_1['normals']
                    )
                    sigma_2, edge_2 = self._compute_sigma_and_edge(
                        self.chain_2['positions'][j], self._kdtree_chain2, self.chain_2['normals']
                    )

                    if not edge_1 and not edge_2:
                        discarded += 1
                        continue

                    if edge_1 and edge_2:
                        side = prev_side if prev_side is not None else 1
                    elif edge_1:
                        side = 1
                    else:
                        side = 2

                    mesh_data = self._mesh_1_data if side == 1 else self._mesh_2_data
                    raw_position = (
                        self.chain_1['positions'][i] if side == 1 else self.chain_2['positions'][j]
                    )

                    if prev_side is not None and side != prev_side:
                        direction = None
                        if len(prev_refined_positions) >= 2:
                            direction = prev_refined_positions[-1] - prev_refined_positions[-2]
                        last_position = (
                            prev_refined_positions[-1] if prev_refined_positions else raw_position
                        )
                        anchor_position, anchor_vertex_idx = self._find_side_switch_anchor(
                            mesh_data, last_position, direction
                        )
                    else:
                        anchor_position = raw_position
                        _, anchor_vertex_idx = mesh_data['vertex_kdtree'].query(anchor_position)
                        anchor_vertex_idx = int(anchor_vertex_idx)

                    refined_position = self._field_scan_refine(
                        mesh_data, anchor_position, anchor_vertex_idx
                    )

                    sp = self._build_seam_point(
                        i, j, refined_position, side, edge_1, edge_2
                    )
                    seam_points.append(sp)

                    prev_side = side
                    prev_refined_positions.append(refined_position)
                    if len(prev_refined_positions) > 2:
                        prev_refined_positions.pop(0)

                except Exception as e:
                    logger.warning(f'Failed at chain_1 index {i}: {e}')
                    failed += 1

            if len(seam_points) >= 2:
                seams.extend(path_creator.process_path(seam_points, self.params))
            elif seam_points:
                logger.debug(
                    f'Dropped seam with only {len(seam_points)} usable point(s)'
                )

        logger.info(
            f'Produced {len(seams)} Seam object(s) '
            f'({failed} point failure(s), {discarded} discarded as neither-edge)'
        )

        return seams
