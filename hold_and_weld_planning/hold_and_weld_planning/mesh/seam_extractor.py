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
                - up_vector: Up direction for edge-to-edge main selection (default [0, 0, 1])
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

        # Up vector for edge-to-edge main/secondary determination
        self.up_vector = np.array(params.get('up_vector', [0, 0, 1]))
        self.up_vector = self.up_vector / np.linalg.norm(self.up_vector)

        # Validate meshes
        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')

        # Initialize path creator
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
            print('Warning: No intersection curve found between meshes')
            return []

        paths = self._chain_segments_to_paths(segments)

        if len(paths) == 0:
            print('Warning: Could not chain segments into paths')
            return []

        seams = []
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
                )

                for geometry in geometries:
                    try:
                        points = geometry['points']

                        # Detect contact type for each mesh
                        is_edge_contact_1, is_edge_contact_2, is_edge_joint = (
                            self._determine_joint_type(points)
                        )

                        # Get normals using contact-type-aware logic
                        normals_1, normals_2 = self._get_normals_for_points(
                            points, is_edge_contact_1, is_edge_contact_2
                        )

                        # Determine main/secondary normals
                        normals_main, normals_secondary = self._determine_main_secondary_normals(
                            normals_1, normals_2, is_edge_joint,
                            is_edge_contact_1, is_edge_contact_2
                        )

                        seam = self._wrap_in_seam(
                            geometry, is_edge_joint, is_edge_contact_1, is_edge_contact_2,
                            normals_main, normals_secondary
                        )
                        seams.append(seam)

                    except Exception as e:
                        print(f'Warning: Failed to process sub-path: {e}')
                        continue

            except Exception as e:
                print(f'Warning: Failed to process path: {e}')
                continue

        return seams

    def _compute_intersection_curve(self) -> np.ndarray:
        """Compute intersection curve between mesh_1 and mesh_2 using CGAL.

        Returns:
            (P, 2, 3) float64 array of intersection edge segments,
            or empty array if no intersection found
        """
        verts1 = np.asarray(self.mesh_1.vertices, dtype=np.float64)
        faces1 = np.asarray(self.mesh_1.faces, dtype=np.int32)
        verts2 = np.asarray(self.mesh_2.vertices, dtype=np.float64)
        faces2 = np.asarray(self.mesh_2.faces, dtype=np.int32)

        try:
            segments = _mesh_intersection.get_intersection_curve(
                verts1, faces1, verts2, faces2, self.inflate
            )
        except RuntimeError as e:
            print(f'Error: CGAL corefinement failed: {e}')
            return np.empty((0, 2, 3))

        return segments

    def _chain_segments_to_paths(self, segments: np.ndarray) -> List[np.ndarray]:
        """Chain intersection edge segments into continuous ordered point paths.

        Args:
            segments: (P, 2, 3) array of edge segments

        Returns:
            List of (N, 3) point arrays, one per continuous path
        """
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
                    if nkey != prev_key and nkey not in visited:
                        next_key = nkey
                        next_pt = npt
                        break

                if next_key is None:
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

        return paths

    def _determine_joint_type(
        self,
        seam_points: np.ndarray,
    ) -> Tuple[bool, bool, bool]:
        """Determine joint type by checking geometry with statistical analysis.

        Args:
            seam_points: Points along the seam (N, 3)

        Returns:
            Tuple of (is_edge_contact_1, is_edge_contact_2, is_edge_joint)
        """
        is_edge_contact_1 = self._is_seam_on_mesh_edge(seam_points, self.mesh_1)
        is_edge_contact_2 = self._is_seam_on_mesh_edge(seam_points, self.mesh_2)
        is_edge_joint = is_edge_contact_1 and is_edge_contact_2

        return is_edge_contact_1, is_edge_contact_2, is_edge_joint

    def _is_seam_on_mesh_edge(
        self,
        seam_points: np.ndarray,
        mesh: trimesh.Trimesh,
    ) -> bool:
        """Check if seam lies on geometric edge via bimodality coefficient.

        Uses Gaussian weighting to prioritize middle points and bimodality
        coefficient to detect one trend (surface) vs two trends (edge).

        Bimodality coefficient = (skewness^2 + 1) / (kurtosis + 3)
        Values > 0.555 indicate bimodal distribution (edge)
        Values < 0.555 indicate unimodal distribution (surface)

        Args:
            seam_points: Points along seam (N, 3)
            mesh: Mesh to check against

        Returns:
            True if seam is on edge (based on weighted bimodality score)
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
                edge_scores.append(0.0)
                continue

            nearby_normals = mesh.face_normals[nearby_faces]
            ref_normal = nearby_normals[0]
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
                bimodality_coeff = (skewness**2 + 1) / (kurtosis + 3)

                # Convert to edge score [0, 1]
                if bimodality_coeff < 0.4:
                    edge_score = 0.0
                elif bimodality_coeff > 0.7:
                    edge_score = 1.0
                else:
                    edge_score = (bimodality_coeff - 0.4) / 0.3

                edge_scores.append(edge_score)

            except Exception:
                edge_scores.append(0.0)

        edge_scores = np.array(edge_scores)
        weighted_score = np.sum(weights * edge_scores) / np.sum(weights)

        return weighted_score > self.edge_detection_threshold

    def _compute_gaussian_weights(self, num_points: int) -> np.ndarray:
        """Compute Gaussian weights centered on middle of seam.

        Args:
            num_points: Number of points along seam

        Returns:
            Array of weights (num_points,) with peak at center
        """
        if num_points < 3:
            return np.ones(num_points)

        center = (num_points - 1) / 2.0
        sigma = num_points * self.gaussian_sigma_ratio
        indices = np.arange(num_points)
        weights = np.exp(-0.5 * ((indices - center) / sigma) ** 2)
        weights = weights * (num_points / np.sum(weights))

        return weights

    def _get_normals_for_points(
        self,
        points: np.ndarray,
        is_edge_contact_1: bool,
        is_edge_contact_2: bool
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get normals for both meshes at seam points.

        Args:
            points: Seam points (N, 3)
            is_edge_contact_1: Whether seam is on mesh_1's edge
            is_edge_contact_2: Whether seam is on mesh_2's edge

        Returns:
            (normals_1, normals_2) each (N, 3)
        """
        normals_1 = []
        normals_2 = []

        for i, point in enumerate(points):
            tangent = self._compute_tangent(points, i)

            if is_edge_contact_1 and is_edge_contact_2:
                # Edge-to-edge: get both faces at each edge
                face_1a, face_1b = self._get_face_candidates_at_point(point, self.mesh_1)
                face_2a, face_2b = self._get_face_candidates_at_point(point, self.mesh_2)

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
                _, _, face_id_2 = trimesh.proximity.closest_point(self.mesh_2, [point])
                normal_2 = self.mesh_2.face_normals[face_id_2[0]]

                face_1a, face_1b = self._get_face_candidates_at_point(point, self.mesh_1)
                normal_1a = self.mesh_1.face_normals[face_1a]
                normal_1b = self.mesh_1.face_normals[face_1b]

                # Pick edge face not antiparallel to surface
                dot_1a = np.dot(normal_1a, normal_2)
                dot_1b = np.dot(normal_1b, normal_2)
                normal_1 = normal_1a if dot_1a > dot_1b else normal_1b

            elif is_edge_contact_2:
                # Mesh 2 edge, Mesh 1 surface
                _, _, face_id_1 = trimesh.proximity.closest_point(self.mesh_1, [point])
                normal_1 = self.mesh_1.face_normals[face_id_1[0]]

                face_2a, face_2b = self._get_face_candidates_at_point(point, self.mesh_2)
                normal_2a = self.mesh_2.face_normals[face_2a]
                normal_2b = self.mesh_2.face_normals[face_2b]

                # Pick edge face not antiparallel to surface
                dot_2a = np.dot(normal_2a, normal_1)
                dot_2b = np.dot(normal_2b, normal_1)
                normal_2 = normal_2a if dot_2a > dot_2b else normal_2b

            else:
                # Surface-to-surface
                _, _, face_id_1 = trimesh.proximity.closest_point(self.mesh_1, [point])
                normal_1 = self.mesh_1.face_normals[face_id_1[0]]

                _, _, face_id_2 = trimesh.proximity.closest_point(self.mesh_2, [point])
                normal_2 = self.mesh_2.face_normals[face_id_2[0]]

            # Project to tangent plane
            normal_1 = self._project_normal_to_tangent_plane(normal_1, tangent)
            normal_2 = self._project_normal_to_tangent_plane(normal_2, tangent)

            normals_1.append(normal_1)
            normals_2.append(normal_2)

        normals_1 = np.array(normals_1)
        normals_2 = np.array(normals_2)

        # Remove outliers
        normals_1 = self._reject_normal_outliers_weighted(normals_1)
        normals_2 = self._reject_normal_outliers_weighted(normals_2)

        return normals_1, normals_2

    def _get_face_candidates_at_point(
        self,
        point: np.ndarray,
        mesh: trimesh.Trimesh
    ) -> Tuple[int, int]:
        """Get two faces sharing the edge closest to this point.

        Args:
            point: 3D point on seam (3,)
            mesh: Mesh to query

        Returns:
            (face_id_1, face_id_2) - Two faces sharing the closest edge
        """
        _, _, face_id = trimesh.proximity.closest_point(mesh, [point])
        closest_face_id = face_id[0]

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
        """Calculate minimum distance from point to line segment.

        Args:
            point: 3D point [x, y, z]
            seg_start: Segment start [x, y, z]
            seg_end: Segment end [x, y, z]

        Returns:
            Distance in meters
        """
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
        is_edge_joint: bool,
        is_edge_contact_1: bool,
        is_edge_contact_2: bool,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Determine which normals are main vs secondary.

        Args:
            normals_1: Normals from mesh_1 (N, 3)
            normals_2: Normals from mesh_2 (N, 3)
            is_edge_joint: Whether edge-to-edge joint
            is_edge_contact_1: Whether mesh_1 is edge
            is_edge_contact_2: Whether mesh_2 is edge

        Returns:
            (normals_main, normals_secondary) each (N, 3)
        """
        if is_edge_joint:
            # Edge-to-edge: use UP vector to pick main
            avg_normal_1 = np.mean(normals_1, axis=0)
            avg_normal_2 = np.mean(normals_2, axis=0)

            avg_normal_1 = avg_normal_1 / np.linalg.norm(avg_normal_1)
            avg_normal_2 = avg_normal_2 / np.linalg.norm(avg_normal_2)

            dot_1 = np.dot(avg_normal_1, self.up_vector)
            dot_2 = np.dot(avg_normal_2, self.up_vector)

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
        """Reject normals that don't fit the trend using Gaussian weighting.

        Args:
            normals: Array of normals (N, 3)

        Returns:
            Cleaned normals array (N, 3)
        """
        if len(normals) < 3:
            return normals

        weights = self._compute_gaussian_weights(len(normals))
        weighted_sum = np.sum(weights[:, np.newaxis] * normals, axis=0)
        avg_normal = weighted_sum / np.sum(weights)

        norm = np.linalg.norm(avg_normal)
        if norm < 1e-10:
            return normals

        avg_normal = avg_normal / norm
        dots = np.einsum('ij,j->i', normals, avg_normal)

        mean_dot = np.sum(weights * dots) / np.sum(weights)
        variance = np.sum(weights * (dots - mean_dot) ** 2) / np.sum(weights)
        std_dot = np.sqrt(variance)

        if std_dot < 1e-10:
            return normals

        deviation = np.abs(dots - mean_dot)
        outlier_mask = deviation > (self.normal_outlier_threshold_std * std_dot)

        cleaned_normals = normals.copy()
        cleaned_normals[outlier_mask] = avg_normal

        return cleaned_normals

    def _compute_tangent(self, points: np.ndarray, index: int) -> np.ndarray:
        """Compute tangent vector at point index.

        Args:
            points: Array of points (N, 3)
            index: Index of point

        Returns:
            Normalized tangent vector
        """
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
        """Project normal into plane perpendicular to tangent.

        Args:
            normal: Raw face normal
            tangent: Normalized tangent vector along seam

        Returns:
            Projected and normalized normal
        """
        projected = normal - np.dot(normal, tangent) * tangent
        norm = np.linalg.norm(projected)

        if norm < 1e-10:
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
        """Wrap path data into Seam object.

        Args:
            geometry: Geometry dict from PathCreator with 'type', 'points', etc.
            is_edge_joint: Whether edge-on-edge joint
            is_edge_contact_1: Whether mesh_1 has edge contact
            is_edge_contact_2: Whether mesh_2 has edge contact
            normals_main: Main normals (N, 3)
            normals_secondary: Secondary normals (N, 3)

        Returns:
            Seam object with LineSegment or ArcSegment

        Raises:
            ValueError: If geometry type is unsupported
        """
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
