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
        on_edge_1:         True if mesh_1 contributes an edge contact.
        on_edge_2:         True if mesh_2 contributes an edge contact.
    """

    position: NDArray
    normal_main: NDArray
    normal_secondary: NDArray
    on_edge_1: bool
    on_edge_2: bool


class SeamExtractor:
    """Extract weld seam points from two touching mesh shells.

    Calls C++ get_seam_vertices, refines points toward the geometric edge via
    covariance-guided stepping, classifies edge contact, and delegates to PathCreator.
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
                    edge_threshold, step_size, max_steps, pairing_radius, num_smooth_points.

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
        self.covariance_radius = params.get('covariance_radius', 0.05)
        self.edge_threshold = params.get('edge_threshold', 0.3)
        self.step_size = params.get('step_size', 0.001)
        self.max_steps = params.get('max_steps', 50)
        self.pairing_radius = params.get('pairing_radius', 0.01)
        self.num_smooth_points = params.get('num_smooth_points', 100)

        self.chain_1: Optional[Dict[str, NDArray]] = None
        self.chain_2: Optional[Dict[str, NDArray]] = None
        self.pairs: Dict[int, int] = {}

        self._seam_1_slices: List[Tuple[int, int, bool]] = []

        self._kdtree_mesh2_verts: Optional[KDTree] = None
        self._kdtree_chain1: Optional[KDTree] = None
        self._kdtree_chain2: Optional[KDTree] = None

        self._kdtree_combined: Optional[KDTree] = None
        self._combined_normals: Optional[NDArray] = None

        self._centroid_1 = np.array(mesh_1.centroid)
        self._centroid_2 = np.array(mesh_2.centroid)

    def _get_local_normals(
        self,
        point: NDArray,
        tree: KDTree,
        normals: NDArray,
    ) -> NDArray:
        """Return face normals within covariance_radius of point from tree/normals."""
        idxs = tree.query_ball_point(point, self.covariance_radius)
        if len(idxs) == 0:
            return np.empty((0, 3))
        return normals[np.array(idxs)]

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

    def _refine_edge_point(self, index: int) -> NDArray:
        """Walk chain_1[index] toward the geometric edge via tangent-plane stepping on mesh_2.

        Steps using position covariance of mesh_2 vertices to stay on the tangent plane.
        Stops when normal covariance σ = λ2/sum of the combined boundary pool peaks,
        indicating the two normal populations meet at the seam. Returns the position p*
        that maximised σ.

        Args:
            index: Index into chain_1.

        Returns:
            Refined position p* (3,).
        """
        p = self.chain_1['positions'][index].copy()

        mesh2_verts = np.asarray(self.mesh_2.vertices, dtype=np.float64)

        _, j0 = self._kdtree_chain2.query(p)
        q0 = self.chain_2['positions'][j0]
        if np.linalg.norm(q0 - p) < 1e-10:
            logger.debug(
                f'Chain_1 index {index}: chain_1 and chain_2 points coincide, '
                'skipping refinement'
            )
            return p

        best_sigma = -np.inf
        best_p = p.copy()

        for step in range(self.max_steps):
            _, j = self._kdtree_chain2.query(p)
            q = self.chain_2['positions'][j]
            u_global = q - p
            u_global_norm = np.linalg.norm(u_global)

            if u_global_norm < 1e-10:
                logger.debug(
                    f'Chain_1 index {index}: reached chain_2 point at step {step}'
                )
                best_p = p.copy()
                break

            u_global = u_global / u_global_norm

            idxs_pos = self._kdtree_mesh2_verts.query_ball_point(p, self.covariance_radius)

            if len(idxs_pos) >= 3:
                pos_data = mesh2_verts[np.array(idxs_pos)]
                cov_pos = np.cov(pos_data.T)  # (3, 3)
                _, pos_eigenvectors = np.linalg.eigh(cov_pos)

                v0 = pos_eigenvectors[:, 0]

                u_tangent = u_global - np.dot(u_global, v0) * v0
                u_tangent_norm = np.linalg.norm(u_tangent)

                if u_tangent_norm > 1e-10:
                    u_tangent = u_tangent / u_tangent_norm
                else:
                    u_tangent = u_global
            else:
                u_tangent = u_global

            p = p + self.step_size * u_tangent

            eigenvalues, _ = self._compute_local_covariance(
                p, self._kdtree_combined, self._combined_normals
            )

            if eigenvalues is None:
                continue

            total = np.sum(eigenvalues)
            if total < 1e-10:
                continue

            sigma = eigenvalues[2] / total

            if sigma > best_sigma:
                best_sigma = sigma
                best_p = p.copy()

            if sigma >= self.edge_threshold:
                logger.debug(
                    f'Chain_1 index {index}: edge found at step {step}, σ={sigma:.3f}'
                )
                break
        else:
            logger.debug(
                f'Chain_1 index {index}: max_steps reached, '
                f'returning best σ={best_sigma:.3f} position'
            )

        return best_p

    def _determine_main_secondary(
        self,
        normal_1: NDArray,
        normal_2: NDArray,
        point: NDArray,
    ) -> Tuple[NDArray, NDArray]:
        """Return (normal_main, normal_secondary) ordered by centroid-to-seam alignment."""
        vec_1 = point - self._centroid_1
        vec_2 = point - self._centroid_2

        norm_v1 = np.linalg.norm(vec_1)
        norm_v2 = np.linalg.norm(vec_2)

        if norm_v1 < 1e-10 or norm_v2 < 1e-10:
            logger.warning('Degenerate centroid-to-seam vector, defaulting to mesh_1 as main')
            return normal_1, normal_2

        dot_1 = np.dot(normal_1, vec_1 / norm_v1)
        dot_2 = np.dot(normal_2, vec_2 / norm_v2)

        if dot_1 >= dot_2:
            return normal_1, normal_2
        else:
            return normal_2, normal_1

    def _build_seam_point(
        self,
        i: int,
        j: int,
        refined_position: NDArray,
    ) -> SeamPoint:
        """Assemble a SeamPoint from chain indices and the refined position."""
        normal_1 = self.chain_1['normals'][i]
        normal_2 = self.chain_2['normals'][j]

        ev_1, _ = self._compute_local_covariance(
            self.chain_1['positions'][i],
            self._kdtree_chain1,
            self.chain_1['normals'],
        )
        ev_2, _ = self._compute_local_covariance(
            self.chain_2['positions'][j],
            self._kdtree_chain2,
            self.chain_2['normals'],
        )

        on_edge_1 = self._is_edge_contact(ev_1) if ev_1 is not None else False
        on_edge_2 = self._is_edge_contact(ev_2) if ev_2 is not None else False

        normal_main, normal_secondary = self._determine_main_secondary(
            normal_1, normal_2, refined_position
        )

        return SeamPoint(
            position=refined_position,
            normal_main=normal_main,
            normal_secondary=normal_secondary,
            on_edge_1=on_edge_1,
            on_edge_2=on_edge_2,
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

        self._kdtree_mesh2_verts = KDTree(verts2)

        self._kdtree_chain1 = KDTree(self.chain_1['positions'])

        combined_positions = np.vstack([
            self.chain_1['positions'],
            self.chain_2['positions'],
        ])
        self._combined_normals = np.vstack([
            self.chain_1['normals'],
            self.chain_2['normals'],
        ])
        self._kdtree_combined = KDTree(combined_positions)

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

        for start, end, _is_closed in self._seam_1_slices:
            seam_points: List[SeamPoint] = []

            for i in range(start, end):
                j = self.pairs.get(i)
                if j is None:
                    continue
                try:
                    p_refined = self._refine_edge_point(i)
                    sp = self._build_seam_point(i, j, p_refined)
                    seam_points.append(sp)
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
            f'Produced {len(seams)} Seam object(s) ({failed} point failure(s))'
        )

        return seams
