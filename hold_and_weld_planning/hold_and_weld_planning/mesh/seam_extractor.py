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

"""SeamExtractor - Extract and classify weld seams from mesh pairs.

This module uses CGAL corefinement (via pybind11) to extract the exact
intersection curve between two meshes, chains the resulting edge segments
into continuous paths, detects geometry type, and wraps results in Seam
objects ready for weld planning.
"""

from collections import defaultdict
import logging
from typing import Dict, List, Tuple
import warnings

import numpy as np
from scipy import stats
import trimesh

from .path_creator import PathCreator
from ..core.arc_segment import ArcSegment
from ..core.line_segment import LineSegment
from ..core.seam import Seam

try:
    from . import mesh_intersection as _mesh_intersection
except ImportError as e:
    raise ImportError(
        'mesh_intersection C++ module not found. '
        'Build it with: cd mesh_intersection && mkdir build && cd build && cmake .. && make'
    ) from e

logger = logging.getLogger(__name__)


class SeamExtractor:
    """Extract weld seams from two touching mesh shells.

    Takes two meshes, computes their geometric intersection curve using CGAL,
    chains the curve into continuous paths, determines joint type
    (edge-on-edge vs edge-on-surface), and wraps results in Seam objects.
    """

    def __init__(
        self, mesh_1: trimesh.Trimesh, mesh_2: trimesh.Trimesh, params: Dict
    ) -> None:
        """Initialize seam extractor with two meshes.

        Args:
            mesh_1: First mesh (already transformed to world frame)
            mesh_2: Second mesh (already transformed to world frame)
            params: Dictionary with keys:
                - inflate: Scale factor for mesh inflation (default 1.002)
                - num_smooth_points: Points per smoothed path (default 100)
                - min_segment_length: Minimum path length to keep (default 5)
                - edge_detection_search_radius_ratio: Search radius as ratio of bbox (default 0.05)
                - edge_detection_threshold: Edge score threshold (default 0.5)
                - gaussian_sigma_ratio: Gaussian weight width (default 0.3)
                - normal_outlier_threshold_std: Outlier rejection threshold (default 2.0)
                - PathCreator config parameters (see PathCreator.process_path docstring
                    for full list):
                    - outlier_std_threshold, min_sub_path_length, min_points_for_corner_detection,
                    - corner_min_angle, corner_angle_window, corner_curvature_window, etc.
        """
        self.mesh_1 = mesh_1
        self.mesh_2 = mesh_2
        self.params = params

        # Core parameters
        self.inflate = params.get('inflate', 1.002)
        self.num_smooth_points = params.get('num_smooth_points', 100)
        self.min_segment_length = params.get('min_segment_length', 5)

        # Edge detection parameters
        self.edge_detection_search_radius_ratio = params.get('edge_detection_search_radius_ratio',
                                                             0.05)
        self.edge_detection_threshold = params.get('edge_detection_threshold', 0.5)
        self.gaussian_sigma_ratio = params.get('gaussian_sigma_ratio', 0.3)
        self.normal_outlier_threshold_std = params.get('normal_outlier_threshold_std', 2.0)

        # Centroids for geometry-based main/secondary determination
        self.centroid_1 = np.array(mesh_1.centroid)
        self.centroid_2 = np.array(mesh_2.centroid)

        path_creator_keys = [
            'outlier_std_threshold',
            'min_sub_path_length',
            'min_points_for_corner_detection',
            'corner_min_angle',
            'corner_angle_window',
            'corner_curvature_window',
            'corner_curvature_threshold',
            'corner_min_agreement',
            'corner_tolerance',
            'corner_filter_window',
            'arc_radius_tolerance',
            'arc_merge_num_points',
            'line_angle_tolerance_deg',
            'line_merge_max_error',
            'line_merge_max_iterations',
            'min_line_size',
            'line_absorption_max_error',
            'line_absorption_max_iterations',
            'first_last_angle_tolerance',
            'first_last_radius_tolerance',
            'first_last_gap_threshold',
        ]
        self.path_creator_config = {k: params[k] for k in path_creator_keys if k in params}

        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')

        self.path_creator = PathCreator()

    def extract_seams(self) -> List[Seam]:
        """Extract all weld seams from the two meshes.

        Pipeline:
        1. Compute intersection curve via CGAL corefinement
        2. Chain edge segments into continuous paths
        3. Smooth paths and detect geometry
        4. Detect contact type (edge vs surface) per mesh
        5. Get normals using correct face logic
        6. Determine main/secondary normals
        7. Wrap in Seam objects

        Returns:
            List of Seam objects with geometry and metadata
        """
        segments = self._compute_intersection_curve()

        if len(segments) == 0:
            logger.warning('No intersection curve found between meshes')
            return []

        logger.info(f'Computed intersection curve: {len(segments)} edge segment(s)')

        paths = self._chain_segments_to_paths(segments)

        if len(paths) == 0:
            logger.warning('Could not chain segments into paths')
            return []

        logger.info(f'Chained into {len(paths)} continuous path(s)')

        seams = []
        paths_succeeded = 0
        paths_failed = 0
        for path_points in paths:
            try:
                is_closed = (
                    len(path_points) > 2 and
                    np.linalg.norm(path_points[0] - path_points[-1]) < 1e-6
                )

                geometries = self.path_creator.process_path(
                    path_points,
                    is_closed=is_closed,
                    num_points=self.num_smooth_points,
                    config=self.path_creator_config,
                )

                for geometry in geometries:
                    try:
                        points = geometry['points']

                        is_edge_contact_1 = self._is_seam_on_mesh_edge(points, self.mesh_1)
                        is_edge_contact_2 = self._is_seam_on_mesh_edge(points, self.mesh_2)
                        is_edge_joint = is_edge_contact_1 and is_edge_contact_2

                        joint_type_str = (
                            'edge-to-edge' if is_edge_joint
                            else f'edge-to-surface '
                                 f"(mesh_1={'edge' if is_edge_contact_1 else 'surface'}, "
                                 f"mesh_2={'edge' if is_edge_contact_2 else 'surface'})"
                        )
                        logger.debug(f'Detected joint type: {joint_type_str}')

                        normals_1, normals_2 = self._get_normals_for_points(
                            points, is_edge_contact_1, is_edge_contact_2
                        )

                        normals_main, normals_secondary = self._determine_main_secondary_normals(
                            normals_1, normals_2, points, is_edge_joint,
                            is_edge_contact_1, is_edge_contact_2
                        )

                        seam = self._wrap_in_seam(
                            geometry, is_edge_joint, is_edge_contact_1, is_edge_contact_2,
                            normals_main, normals_secondary
                        )
                        seams.append(seam)

                    except Exception as e:
                        logger.warning(f'Failed to process sub-path: {e}')
                        continue

                paths_succeeded += 1

            except Exception as e:
                logger.warning(f'PathCreator.process_path() failed for path: {e}')
                paths_failed += 1
                continue

        logger.info(f'Extracted {len(seams)} seam(s) from {len(paths)} path(s) '
                    f'({paths_succeeded} succeeded, {paths_failed} failed)')
        return seams

    def _compute_intersection_curve(self) -> np.ndarray:
        """Compute intersection curve between mesh_1 and mesh_2 using CGAL."""
        verts1 = np.asarray(self.mesh_1.vertices, dtype=np.float64)
        faces1 = np.asarray(self.mesh_1.faces, dtype=np.int32)
        verts2 = np.asarray(self.mesh_2.vertices, dtype=np.float64)
        faces2 = np.asarray(self.mesh_2.faces, dtype=np.int32)

        try:
            segments = _mesh_intersection.get_intersection_curve(
                verts1, faces1, verts2, faces2, self.inflate
            )
        except RuntimeError as e:
            logger.error(f'CGAL corefinement failed: {e}')
            return np.empty((0, 2, 3))

        return segments

    def _chain_segments_to_paths(self, segments: np.ndarray) -> List[np.ndarray]:
        """Chain intersection edge segments into continuous ordered point paths."""
        if len(segments) == 0:
            return []

        tol = 1e-6
        rounded = np.round(segments / tol).astype(np.int64)

        adjacency = defaultdict(list)

        for i in range(len(segments)):
            p0_key = tuple(rounded[i, 0])
            p1_key = tuple(rounded[i, 1])
            p0 = segments[i, 0]
            p1 = segments[i, 1]

            adjacency[p0_key].append((p1_key, p1))
            adjacency[p1_key].append((p0_key, p0))

        visited = set()
        paths = []

        # Warn about branching nodes (degree > 2) — these are T-junctions in the intersection
        # curve. The greedy traversal will pick one branch and silently drop the others.
        branching_nodes = [k for k, v in adjacency.items() if len(v) > 2]
        if branching_nodes:
            logger.warning(
                f'{len(branching_nodes)} branching node(s) detected in intersection graph '
                f'(T-junctions) — only one branch will be followed per node, others are dropped'
            )

        for start_key in adjacency.keys():
            if start_key in visited:
                continue

            path_points = [
                np.array([start_key[0] * tol, start_key[1] * tol, start_key[2] * tol])
            ]
            visited.add(start_key)
            current_key = start_key
            prev_key = None

            while True:
                neighbors = adjacency[current_key]

                next_key = None
                next_pt = None
                for nkey, npt in neighbors:
                    if nkey != prev_key and nkey not in visited:  # skip back-edge
                        next_key = nkey
                        next_pt = npt
                        break

                if next_key is None:
                    # Path ended - check if it loops back to start
                    for nkey, npt in neighbors:
                        if nkey == start_key and len(path_points) > 2:
                            path_points.append(path_points[0].copy())
                            break
                    break

                path_points.append(next_pt)
                visited.add(next_key)
                prev_key = current_key
                current_key = next_key

            if len(path_points) >= self.min_segment_length:
                paths.append(np.array(path_points))

        logger.debug(f'Chained {len(segments)} segment(s) into {len(paths)} path(s), '
                     f'{sum(len(p) for p in paths)} total points')

        return paths

    def _is_seam_on_mesh_edge(
        self,
        seam_points: np.ndarray,
        mesh: trimesh.Trimesh,
    ) -> bool:
        """Check if seam lies on geometric edge using bimodality coefficient of face normals.

        Searches for triangle mesh faces near each seam point. Edge contact shows bimodal
        normal distribution (two surfaces meeting), surface contact shows unimodal distribution.
        Bimodality coefficient (BC) = (skew^2 + 1) / (kurtosis + 3).
        BC > 0.555 indicates bimodal (edge), BC < 0.555 indicates unimodal (surface).
        Gaussian weights prioritize center points for robustness.
        """
        bbox_size = np.linalg.norm(mesh.bounds[1] - mesh.bounds[0])
        search_radius = bbox_size * self.edge_detection_search_radius_ratio

        weights = self._compute_gaussian_weights(len(seam_points))
        edge_scores = []

        for point in seam_points:
            distances = np.linalg.norm(mesh.vertices - point, axis=1)
            nearby_mask = distances < search_radius
            nearby_vertex_indices = np.where(nearby_mask)[0]

            if len(nearby_vertex_indices) < 3:
                edge_scores.append(0.0)
                continue

            nearby_faces = set()
            for v_idx in nearby_vertex_indices:
                faces_with_v = np.where(np.any(mesh.faces == v_idx, axis=1))[0]
                nearby_faces.update(faces_with_v.tolist())

            nearby_faces = list(nearby_faces)

            if len(nearby_faces) < 4:
                # Need at least 4 triangle mesh faces for reliable bimodality statistics
                edge_scores.append(0.0)
                continue

            nearby_normals = mesh.face_normals[nearby_faces]
            ref_normal = nearby_normals[0]
            # Project all normals onto a reference to get a scalar distribution.
            # Bimodal distribution -> two distinct normal clusters -> edge contact.
            dots = np.einsum('ij,j->i', nearby_normals, ref_normal)

            if len(np.unique(dots)) < 2:
                edge_scores.append(0.0)
                continue

            try:
                with warnings.catch_warnings():
                    warnings.filterwarnings('ignore', message='Precision '
                                            'loss occurred in moment calculation')

                    skewness = stats.skew(dots)
                    kurtosis = stats.kurtosis(dots)

                # BC = (skewness^2 + 1) / (kurtosis + 3)
                # TODO(@silanus23): Replace brute-force vertex search with
                # KDTree built once per mesh
                denominator = max(kurtosis + 3, 1e-10)
                bimodality_coeff = (skewness**2 + 1) / denominator

                # Convert to edge score [0, 1]
                if bimodality_coeff < 0.4:
                    edge_score = 0.0
                elif bimodality_coeff > 0.7:
                    edge_score = 1.0
                else:
                    edge_score = (bimodality_coeff - 0.4) / 0.3

                edge_scores.append(edge_score)

            except Exception as e:
                logger.debug(f'Bimodality computation failed at point, scoring as surface: {e}')
                edge_scores.append(0.0)

        edge_scores = np.array(edge_scores)
        weight_sum = np.sum(weights)

        if weight_sum < 1e-10:
            logger.warning('Gaussian weights sum to zero, cannot compute weighted edge score')
            return False

        weighted_score = np.sum(weights * edge_scores) / weight_sum

        return weighted_score > self.edge_detection_threshold

    def _compute_gaussian_weights(self, num_points: int) -> np.ndarray:
        """Compute Gaussian weights centered on middle of seam to prioritize stable center points.

        Weights are centered on the middle of the seam to prioritize stable center points.
        """
        if num_points < 3:
            return np.ones(num_points)

        center = (num_points - 1) / 2.0
        sigma = num_points * self.gaussian_sigma_ratio
        indices = np.arange(num_points)
        weights = np.exp(-0.5 * ((indices - center) / sigma) ** 2)
        # Rescale so weights sum to num_points, keeping weighted stats
        # comparable to unweighted ones.
        weights = weights * (num_points / np.sum(weights))

        return weights

    def _get_normals_for_points(
        self,
        points: np.ndarray,
        is_edge_contact_1: bool,
        is_edge_contact_2: bool
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get normals for both meshes at seam points using contact-type-aware face selection."""
        normals_1 = []
        normals_2 = []

        for i, point in enumerate(points):
            tangent = self._compute_tangent(points, i)

            if is_edge_contact_1 and is_edge_contact_2:
                # Edge-to-edge: get both faces at each edge
                try:
                    face_1a, face_1b = self._get_face_candidates_at_point(point, self.mesh_1)
                except Exception as e:
                    logger.warning(
                        f'Failed to get face candidates on mesh_1: {e},'
                        ' using face index 0 as fallback'
                    )
                    face_1a = face_1b = 0

                try:
                    face_2a, face_2b = self._get_face_candidates_at_point(point, self.mesh_2)
                except Exception as e:
                    logger.warning(
                        f'Failed to get face candidates on mesh_2: {e},'
                        ' using face index 0 as fallback'
                    )
                    face_2a = face_2b = 0

                normal_1a = self.mesh_1.face_normals[face_1a]
                normal_1b = self.mesh_1.face_normals[face_1b]
                normal_2a = self.mesh_2.face_normals[face_2a]
                normal_2b = self.mesh_2.face_normals[face_2b]

                # Pick pair with most parallel normals
                parallel_scores = [
                    (np.dot(normal_1a, normal_2a), normal_1a, normal_2a),
                    (np.dot(normal_1a, normal_2b), normal_1a, normal_2b),
                    (np.dot(normal_1b, normal_2a), normal_1b, normal_2a),
                    (np.dot(normal_1b, normal_2b), normal_1b, normal_2b),
                ]

                _, normal_1, normal_2 = max(parallel_scores, key=lambda x: x[0])

            elif is_edge_contact_1:
                # Mesh 1 edge, Mesh 2 surface
                try:
                    _, _, face_id_2 = trimesh.proximity.closest_point(self.mesh_2, [point])
                    normal_2 = self.mesh_2.face_normals[face_id_2[0]]
                except Exception as e:
                    logger.warning(f'Failed to get closest point on mesh_2: {e}')
                    normal_2 = np.array([0, 0, 1])

                face_1a, face_1b = self._get_face_candidates_at_point(point, self.mesh_1)
                normal_1a = self.mesh_1.face_normals[face_1a]
                normal_1b = self.mesh_1.face_normals[face_1b]

                # Pick edge face not antiparallel to surface
                dot_1a = np.dot(normal_1a, normal_2)
                dot_1b = np.dot(normal_1b, normal_2)
                normal_1 = normal_1a if dot_1a > dot_1b else normal_1b

            elif is_edge_contact_2:
                # Mesh 2 edge, Mesh 1 surface
                try:
                    _, _, face_id_1 = trimesh.proximity.closest_point(self.mesh_1, [point])
                    normal_1 = self.mesh_1.face_normals[face_id_1[0]]
                except Exception as e:
                    logger.warning(f'Failed to get closest point on mesh_1: {e}')
                    normal_1 = np.array([0, 0, 1])

                face_2a, face_2b = self._get_face_candidates_at_point(point, self.mesh_2)
                normal_2a = self.mesh_2.face_normals[face_2a]
                normal_2b = self.mesh_2.face_normals[face_2b]

                # Pick edge face not antiparallel to surface
                dot_2a = np.dot(normal_2a, normal_1)
                dot_2b = np.dot(normal_2b, normal_1)
                normal_2 = normal_2a if dot_2a > dot_2b else normal_2b

            else:
                # Surface-to-surface
                # TODO(@silanus23): Consider deleting this
                try:
                    _, _, face_id_1 = trimesh.proximity.closest_point(self.mesh_1, [point])
                    normal_1 = self.mesh_1.face_normals[face_id_1[0]]
                except Exception as e:
                    logger.warning(f'Failed to get closest point on mesh_1: {e}')
                    normal_1 = np.array([0, 0, 1])

                try:
                    _, _, face_id_2 = trimesh.proximity.closest_point(self.mesh_2, [point])
                    normal_2 = self.mesh_2.face_normals[face_id_2[0]]
                except Exception as e:
                    logger.warning(f'Failed to get closest point on mesh_2: {e}')
                    normal_2 = np.array([0, 0, 1])

            normal_1 = self._project_normal_to_tangent_plane(normal_1, tangent)
            normal_2 = self._project_normal_to_tangent_plane(normal_2, tangent)

            normals_1.append(normal_1)
            normals_2.append(normal_2)

        normals_1 = np.array(normals_1)
        normals_2 = np.array(normals_2)

        normals_1 = self._reject_normal_outliers_weighted(normals_1)
        normals_2 = self._reject_normal_outliers_weighted(normals_2)

        return normals_1, normals_2

    def _get_face_candidates_at_point(
        self,
        point: np.ndarray,
        mesh: trimesh.Trimesh
    ) -> Tuple[int, int]:
        """Get two faces sharing the edge closest to this point."""
        try:
            _, _, face_id = trimesh.proximity.closest_point(mesh, [point])
            closest_face_id = face_id[0]
        except Exception as e:
            logger.warning(
                f'Failed to get closest point on mesh: {e}, returning face 0 as fallback'
            )
            # Caller (_get_normals_for_points) has its own except that substitutes [0,0,1]
            # if the face 0 normal is nonsensical for this point.
            return 0, 0

        face_vertices = mesh.faces[closest_face_id]
        face_edges = [
            (face_vertices[0], face_vertices[1]),
            (face_vertices[1], face_vertices[2]),
            (face_vertices[2], face_vertices[0])
        ]

        min_dist = float('inf')
        closest_edge = None

        for v1, v2 in face_edges:
            edge_start = mesh.vertices[v1]
            edge_end = mesh.vertices[v2]
            dist = self._point_to_line_segment_distance(point, edge_start, edge_end)
            if dist < min_dist:
                min_dist = dist
                closest_edge = (v1, v2)

        if closest_edge is None:
            logger.warning(
                f'No valid edge found in face {closest_face_id}, returning duplicate face'
            )
            return int(closest_face_id), int(closest_face_id)

        v1, v2 = closest_edge
        faces_with_v1 = np.where(np.any(mesh.faces == v1, axis=1))[0]
        faces_with_v2 = np.where(np.any(mesh.faces == v2, axis=1))[0]
        shared_faces = np.intersect1d(faces_with_v1, faces_with_v2)

        if len(shared_faces) >= 2:
            return int(shared_faces[0]), int(shared_faces[1])
        elif len(shared_faces) == 1:
            return int(shared_faces[0]), int(shared_faces[0])
        else:
            return int(closest_face_id), int(closest_face_id)

    def _point_to_line_segment_distance(
        self,
        point: np.ndarray,
        seg_start: np.ndarray,
        seg_end: np.ndarray
    ) -> float:
        """Calculate minimum distance from point to line segment."""
        segment = seg_end - seg_start
        length_sq = np.dot(segment, segment)

        if length_sq < 1e-10:
            return np.linalg.norm(point - seg_start)

        t = np.clip(np.dot(point - seg_start, segment) / length_sq, 0.0, 1.0)
        projection = seg_start + t * segment

        return np.linalg.norm(point - projection)

    def _determine_main_secondary_normals(
        self,
        normals_1: np.ndarray,
        normals_2: np.ndarray,
        points: np.ndarray,
        is_edge_joint: bool,
        is_edge_contact_1: bool,
        is_edge_contact_2: bool,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Determine which normals are main vs secondary based on joint type and part centroids.

        The main surface is the base plate — the one whose normals point *away* from its
        own part centroid relative to the seam. Computed by projecting the centroid-to-seam
        vector onto each part's average normal: the higher dot product means the normal
        aligns with the outward direction -> that part is the base (main).
        """
        if is_edge_joint:
            avg_normal_1 = np.mean(normals_1, axis=0)
            avg_normal_2 = np.mean(normals_2, axis=0)

            norm_1 = np.linalg.norm(avg_normal_1)
            norm_2 = np.linalg.norm(avg_normal_2)

            if norm_1 < 1e-10 or norm_2 < 1e-10:
                logger.warning('Zero-length average normal detected, defaulting to mesh_1 as main')
                return normals_1, normals_2

            avg_normal_1 = avg_normal_1 / norm_1
            avg_normal_2 = avg_normal_2 / norm_2

            # Vector from each part centroid to the seam centroid
            seam_centroid = np.mean(points, axis=0)
            vec_1 = seam_centroid - self.centroid_1
            vec_2 = seam_centroid - self.centroid_2

            norm_v1 = np.linalg.norm(vec_1)
            norm_v2 = np.linalg.norm(vec_2)

            if norm_v1 < 1e-10 or norm_v2 < 1e-10:
                logger.warning('Degenerate centroid-to-seam vector, defaulting to mesh_1 as main')
                return normals_1, normals_2

            vec_1 = vec_1 / norm_v1
            vec_2 = vec_2 / norm_v2

            # The part whose average normal aligns more with its centroid-to-seam vector
            # has the seam on its outward face -> that is the base plate (main)
            dot_1 = np.dot(avg_normal_1, vec_1)
            dot_2 = np.dot(avg_normal_2, vec_2)

            logger.debug(f'Centroid-based main selection: dot_1={dot_1:.3f}, dot_2={dot_2:.3f}')

            if dot_1 >= dot_2:
                return normals_1, normals_2
            else:
                return normals_2, normals_1
        else:
            # Edge-to-surface: surface is main, edge is secondary
            if is_edge_contact_1:
                return normals_2, normals_1
            else:
                return normals_1, normals_2

    def _reject_normal_outliers_weighted(
        self,
        normals: np.ndarray,
    ) -> np.ndarray:
        """Reject normals that don't fit the trend using weighted statistical threshold."""
        if len(normals) < 3:
            return normals

        weights = self._compute_gaussian_weights(len(normals))
        weight_sum = np.sum(weights)

        if weight_sum < 1e-10:
            logger.debug('Weights sum to zero in outlier rejection, skipping')
            return normals

        weighted_sum = np.sum(weights[:, np.newaxis] * normals, axis=0)
        avg_normal = weighted_sum / weight_sum

        norm = np.linalg.norm(avg_normal)
        if norm < 1e-10:
            logger.debug('Average normal has zero length, skipping outlier rejection')
            return normals

        avg_normal = avg_normal / norm
        dots = np.einsum('ij,j->i', normals, avg_normal)

        mean_dot = np.sum(weights * dots) / weight_sum
        variance = np.sum(weights * (dots - mean_dot) ** 2) / weight_sum
        std_dot = np.sqrt(variance)

        if std_dot < 1e-10:
            return normals

        deviation = np.abs(dots - mean_dot)
        outlier_mask = deviation > (self.normal_outlier_threshold_std * std_dot)

        cleaned_normals = normals.copy()
        # Replace outliers with average rather than removing —
        # array must stay same length as points.
        cleaned_normals[outlier_mask] = avg_normal

        num_outliers = np.sum(outlier_mask)
        if num_outliers > 0:
            logger.debug(f'Replaced {num_outliers} outlier normal(s) out of {len(normals)}')

        return cleaned_normals

    def _compute_tangent(self, points: np.ndarray, index: int) -> np.ndarray:
        """Compute normalized tangent vector at point index using forward/backward differences."""
        if index == 0:
            tangent = points[1] - points[0]
        elif index == len(points) - 1:
            tangent = points[-1] - points[-2]
        else:
            tangent = points[index + 1] - points[index - 1]

        norm = np.linalg.norm(tangent)
        if norm < 1e-10:
            return np.array([1, 0, 0])

        return tangent / norm

    def _project_normal_to_tangent_plane(
        self, normal: np.ndarray, tangent: np.ndarray
    ) -> np.ndarray:
        """Project normal into plane perpendicular to tangent and normalize."""
        projected = normal - np.dot(normal, tangent) * tangent
        norm = np.linalg.norm(projected)

        if norm < 1e-10:
            # Normal is parallel to tangent — projection collapses. Return unprojected as fallback.
            return normal

        return projected / norm

    def _wrap_in_seam(
        self,
        geometry: Dict,
        is_edge_joint: bool,
        is_edge_contact_1: bool,
        is_edge_contact_2: bool,
        normals_main: np.ndarray,
        normals_secondary: np.ndarray,
    ) -> Seam:
        """Wrap geometry and normals into Seam object with metadata."""
        points = geometry['points']

        if geometry['type'] == 'line':
            line_segment = LineSegment(start=points[0], end=points[-1])
            seam = Seam(line_segment=line_segment)

        elif geometry['type'] == 'arc':
            arc_segment = ArcSegment(
                points=points,
                center=geometry['center'],
                radius=geometry['radius']
            )
            seam = Seam(arc_segment=arc_segment)

        else:
            logger.error(f'Unknown geometry type: {geometry["type"]}')
            raise ValueError(f'Unknown geometry type: {geometry["type"]}')

        seam.config['is_edge_joint'] = is_edge_joint
        seam.config['mesh_1_is_edge'] = is_edge_contact_1
        seam.config['mesh_2_is_edge'] = is_edge_contact_2
        seam.config['geometry_type'] = geometry['type']
        seam.config['normals_main'] = normals_main
        seam.config['normals_secondary'] = normals_secondary
        seam.config['smoothed_points'] = points
        seam.config['source_mesh'] = 1

        return seam
