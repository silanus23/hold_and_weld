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

import numpy as np
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

            - inflate: Scale factor for mesh inflation to create overlap (default 1.002)
            - num_smooth_points: Points per smoothed path (default 100)
            - min_segment_length: Minimum path length to keep (default 5)

        Args:
            mesh_1: First mesh (already transformed to world frame)
            mesh_2: Second mesh (already transformed to world frame)
            params: Dictionary with keys:
        """
        self.mesh_1 = mesh_1
        self.mesh_2 = mesh_2
        self.params = params
        self.inflate = params.get('inflate', 1.002)
        self.num_smooth_points = params.get('num_smooth_points', 100)
        self.min_segment_length = params.get('min_segment_length', 5)
        self.angle_threshold_deg = params.get('angle_threshold_deg', 45.0)
        self.corner_window = params.get('corner_window', 5)
        self.line_error_threshold = params.get('line_error_threshold', 0.0001)
        self.circle_error_threshold = params.get('circle_error_threshold', 0.0001)

        # Validate meshes
        if not mesh_1.is_watertight:
            raise ValueError('mesh_1 is not watertight')
        if not mesh_2.is_watertight:
            raise ValueError('mesh_2 is not watertight')

        # Initialize path creator for smoothing/geometry detection
        self.path_creator = PathCreator()

    def extract_seams(self) -> List[Seam]:
        """Extract all weld seams from the two meshes.

        Pipeline:
        1. Compute intersection curve via CGAL corefinement
        2. Chain edge segments into continuous paths
        3. Smooth paths and detect geometry
        4. Compute normals once per path, reuse for joint type and seam wrapping
        5. Wrap in Seam objects

        Returns:
            List of Seam objects with geometry and metadata
        """
        segments = self._compute_intersection_curve()

        if len(segments) == 0:
            print('Warning: No intersection curve found between meshes')
            return []

        print(f'DEBUG: Got {len(segments)} intersection edge segments')

        paths = self._chain_segments_to_paths(segments)

        if len(paths) == 0:
            print('Warning: Could not chain segments into paths')
            return []

        print(f'DEBUG: Chained into {len(paths)} paths')

        seams = []
        for path_points in paths:
            try:
                is_closed = (
                    len(path_points) > 2 and
                    np.linalg.norm(path_points[0] - path_points[-1]) < 1e-6
                )

                print(
                    f'DEBUG path: processing {"closed" if is_closed else "open"} '
                    f'path with {len(path_points)} points'
                )

                geometries = self.path_creator.process_path(
                    path_points,
                    is_closed=is_closed,
                    num_points=self.num_smooth_points,
                    angle_threshold_deg=self.angle_threshold_deg,
                    corner_window=self.corner_window,
                    line_error_threshold=self.line_error_threshold,
                    circle_error_threshold=self.circle_error_threshold,
                )

                for geometry in geometries:
                    try:
                        points = geometry['points']
                        normals_1, normals_2 = self._get_normals_for_points(points)
                        is_edge_joint = self._determine_joint_type(normals_1, normals_2)
                        seam = self._wrap_in_seam(
                            geometry, is_edge_joint, normals_1, normals_2
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

        Calls the compiled C++ pybind11 module which:
        1. Inflates mesh_2 slightly to guarantee overlap for touching meshes
        2. Runs CGAL corefinement to extract intersection edges
        3. Returns segments as (P, 2, 3) array

                or empty array if no intersection found

        Returns:
            (P, 2, 3) float64 array of intersection edge segments,
        """
        verts1 = np.asarray(self.mesh_1.vertices, dtype=np.float64)
        faces1 = np.asarray(self.mesh_1.faces, dtype=np.int32)
        verts2 = np.asarray(self.mesh_2.vertices, dtype=np.float64)
        faces2 = np.asarray(self.mesh_2.faces, dtype=np.int32)

        print(f'DEBUG: inflate factor = {self.inflate}')

        try:
            segments = _mesh_intersection.get_intersection_curve(
                verts1, faces1, verts2, faces2, self.inflate
            )
        except RuntimeError as e:
            print(f'DEBUG: CGAL corefinement failed: {e}')
            return np.empty((0, 2, 3))

        print(f'DEBUG: CGAL returned {len(segments)} segments')
        return segments

    def _chain_segments_to_paths(self, segments: np.ndarray) -> List[np.ndarray]:
        """Chain intersection edge segments into continuous ordered point paths.

        Builds a coordinate-based graph from segment endpoints and traverses
        it to form continuous paths. Filters out paths shorter than
        min_segment_length.

        Args:
            segments: (P, 2, 3) array of edge segments
        Returns:
            List of (N, 3) point arrays, one per continuous path
        """
        if len(segments) == 0:
            return []

        # Round coordinates to merge near-duplicate endpoints
        tol = 1e-6
        rounded = np.round(segments / tol).astype(np.int64)

        adjacency = defaultdict(list)

        for i, seg in enumerate(segments):
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

        print(f'DEBUG: Found {len(paths)} paths after filtering')
        for i, p in enumerate(paths):
            print(f'DEBUG: Path {i}: {len(p)} points')

        return paths

    def _determine_joint_type(
        self,
        normals_1: np.ndarray,
        normals_2: np.ndarray,
    ) -> bool:
        """Determine if joint is edge-on-edge or edge-on-surface.

        Compares normals from both meshes at seam points. If normals are
        nearly parallel (dot product > cos(5°) ≈ 0.996), it's edge-on-edge.
        Otherwise it's a T-joint (edge-on-surface).

        Args:
            normals_1: Normals from mesh_1 (N, 3)
            normals_2: Normals from mesh_2 (N, 3)
        Returns:
            True if edge-on-edge joint, False if edge-on-surface (T-joint)
        """
        dots = np.abs(np.einsum('ij,ij->i', normals_1, normals_2))
        avg_dot = np.mean(dots)

        print(f'DEBUG: Normal dot product avg = {avg_dot:.3f}')

        # Normals within 5° of parallel (cos(5°) ≈ 0.996) = edge-on-edge
        is_edge_joint = avg_dot > 0.996

        joint_type = 'edge-on-edge' if is_edge_joint else 'T-joint (edge-on-surface)'
        print(f'DEBUG: Joint type = {joint_type}')
        return is_edge_joint

    def _get_normals_for_points(
        self, points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Get surface normals for each point using tangent-based projection.

        Instead of using raw face normals (which flip at sharp edges),
        projects each face normal into the plane perpendicular to the
        seam tangent. This gives stable, consistent normals regardless
        of which face the closest point query lands on.

        Args:
            points: Array of 3D points (N, 3)
        Returns:
            Tuple of (normals_from_mesh1, normals_from_mesh2)
        """
        _, _, face_ids_1 = trimesh.proximity.closest_point(self.mesh_1, points)
        raw_normals_1 = self.mesh_1.face_normals[face_ids_1]

        _, _, face_ids_2 = trimesh.proximity.closest_point(self.mesh_2, points)
        raw_normals_2 = self.mesh_2.face_normals[face_ids_2]

        normals_1 = np.zeros_like(raw_normals_1)
        normals_2 = np.zeros_like(raw_normals_2)

        for i in range(len(points)):
            if i == 0:
                tangent = points[1] - points[0]
            elif i == len(points) - 1:
                tangent = points[-1] - points[-2]
            else:
                tangent = points[i + 1] - points[i - 1]

            norm = np.linalg.norm(tangent)
            if norm < 1e-10:
                normals_1[i] = raw_normals_1[i]
                normals_2[i] = raw_normals_2[i]
                continue

            tangent = tangent / norm

            # Project normal into plane perpendicular to tangent
            # n_proj = normalize(n - dot(n, t) * t)
            normals_1[i] = self._project_normal_to_tangent_plane(
                raw_normals_1[i], tangent
            )
            normals_2[i] = self._project_normal_to_tangent_plane(
                raw_normals_2[i], tangent
            )

        return normals_1, normals_2

    def _project_normal_to_tangent_plane(
        self, normal: np.ndarray, tangent: np.ndarray
    ) -> np.ndarray:
        """Project normal into plane perpendicular to tangent.

        Removes the tangent component from the normal, giving a stable
        surface normal that is consistent with the path direction.

        Args:
            normal: Raw face normal
            tangent: Normalized tangent vector along seam
        Returns:
            Projected and normalized normal
        """
        projected = normal - np.dot(normal, tangent) * tangent
        norm = np.linalg.norm(projected)

        if norm < 1e-10:
            # Normal is parallel to tangent - fallback to raw
            return normal

        return projected / norm

    def _wrap_in_seam(
        self,
        geometry: Dict,  # ← Changed from path_dict to geometry
        is_edge_joint: bool,
        normals_1: np.ndarray,
        normals_2: np.ndarray,
    ) -> Seam:
        """Wrap path data into Seam object.

        Args:
            geometry: Geometry dict from PathCreator with 'type', 'points', etc.
            is_edge_joint: Whether edge-on-edge joint
            normals_1: Normals from mesh_1
            normals_2: Normals from mesh_2
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
        seam.config['geometry_type'] = geometry['type']
        seam.config['normals_mesh_1'] = normals_1
        seam.config['normals_mesh_2'] = normals_2
        seam.config['smoothed_points'] = points
        seam.config['source_mesh'] = 1

        return seam
