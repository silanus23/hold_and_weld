# Copyright 2025 Berkan Tali
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

"""Surface extractor - extracts planar surfaces from URDF collision geometries.

This module provides tools for extracting planar surfaces from URDF collision
geometries (boxes and cylinders). Surfaces are represented as Surface objects
containing center, normal, axes, bounds, and area information.
"""

from typing import Any, List

import numpy as np
from numpy.typing import NDArray

from ..core.surface import Surface


class SurfaceExtractor:
    """Extract planar surfaces from URDF collision geometries.

    Supports box and cylinder geometries. Returns surfaces as Surface objects
    with world-frame coordinates.
    """

    def __init__(self, urdf_processor: Any) -> None:
        """Initialize surface extractor.

        Args:
            urdf_processor: URDFProcessor instance used to retrieve link and
                transformation data.
        """
        self.urdf_processor = urdf_processor

    def extract_all_surfaces_from_links(
        self, link_names: List[str]
    ) -> List[Surface]:
        """Extract all planar surfaces from multiple links.

        Args:
            link_names: List of link names to process.

        Returns:
            List of Surface objects from all links.
        """
        all_surfaces = []

        for link_name in link_names:
            surfaces = self.extract_all_surfaces(link_name)
            all_surfaces.extend(surfaces)

        return all_surfaces

    def extract_all_surfaces(self, link_name: str) -> List[Surface]:
        """Extract all planar surfaces from a link's collision geometries.

        Args:
            link_name: Name of the link to analyze.

        Returns:
            List of Surface objects, each containing:
                - surface_id: identifier string (e.g., 'link:0:top')
                - center: np.array [x, y, z] in world frame
                - normal: np.array [x, y, z] unit vector in world frame
                - corners: list of corner points (N corners for N-gon)
                - edges: list of (start, end) point pairs
                - u_axis: np.array [x, y, z] first basis vector
                - v_axis: np.array [x, y, z] second basis vector
                - bounds: dict with u_min, u_max, v_min, v_max keys

        Raises:
            ValueError: If link not found or geometry not supported.
        """
        link = self.urdf_processor.get_link(link_name)

        if not link.collisions and not link.collision:
            raise ValueError(f"Link '{link_name}' has no collision geometry")

        # Handle both single collision and multiple collisions
        collisions = link.collisions if link.collisions else [link.collision]

        all_surfaces = []

        for collision_idx, collision in enumerate(collisions):
            if collision is None:
                continue

            geometry = collision.geometry

            local_transform = self.urdf_processor.get_collision_transform(collision)
            world_transform = self.urdf_processor.world_transform @ local_transform

            if hasattr(geometry, 'size'):
                # Box geometry
                surfaces = self._extract_box_surfaces(
                    geometry, world_transform, link_name, collision_idx
                )
            elif hasattr(geometry, 'radius') and hasattr(geometry, 'length'):
                # Cylinder geometry
                surfaces = self._extract_cylinder_surfaces(
                    geometry, world_transform, link_name, collision_idx
                )
            else:
                raise ValueError(
                    f"Link '{link_name}' collision {collision_idx}: "
                    f'unsupported geometry type (only box and cylinder supported)'
                )

            all_surfaces.extend(surfaces)

        return all_surfaces

    def _extract_box_surfaces(
        self,
        geometry: Any,
        world_transform: NDArray,
        link_name: str,
        collision_idx: int
    ) -> List[Surface]:
        """Extract 6 planar surfaces from a box geometry.

        Args:
            geometry: Box geometry with 'size' attribute.
            world_transform: 4x4 transformation matrix to world frame.
            link_name: Name of the parent link.
            collision_idx: Index of collision within link.

        Returns:
            List of 6 Surface objects.
        """
        length, width, height = geometry.size

        local_faces = {
            'top': {
                'offset': [0, 0, height/2], 'normal': [0, 0, 1],
                'u_axis': [1, 0, 0], 'v_axis': [0, 1, 0], 'dims': [length, width]
            },
            'bottom': {
                'offset': [0, 0, -height/2], 'normal': [0, 0, -1],
                'u_axis': [1, 0, 0], 'v_axis': [0, -1, 0], 'dims': [length, width]
            },
            'front': {
                'offset': [length/2, 0, 0], 'normal': [1, 0, 0],
                'u_axis': [0, 1, 0], 'v_axis': [0, 0, 1], 'dims': [width, height]
            },
            'back': {
                'offset': [-length/2, 0, 0], 'normal': [-1, 0, 0],
                'u_axis': [0, -1, 0], 'v_axis': [0, 0, 1], 'dims': [width, height]
            },
            'right': {
                'offset': [0, width/2, 0], 'normal': [0, 1, 0],
                'u_axis': [1, 0, 0], 'v_axis': [0, 0, 1], 'dims': [length, height]
            },
            'left': {
                'offset': [0, -width/2, 0], 'normal': [0, -1, 0],
                'u_axis': [-1, 0, 0], 'v_axis': [0, 0, 1], 'dims': [length, height]
            },
        }

        return self._transform_faces_to_world(
            local_faces, world_transform, link_name, collision_idx
        )

    def _extract_cylinder_surfaces(
        self,
        geometry: Any,
        world_transform: NDArray,
        link_name: str,
        collision_idx: int
    ) -> List[Surface]:
        """Extract 2 planar cap surfaces from a cylinder geometry.

        Args:
            geometry: Cylinder geometry with radius and length.
            world_transform: 4x4 transformation matrix to world frame.
            link_name: Name of the parent link.
            collision_idx: Index of collision within link.

        Returns:
            List of 2 Surface objects (top and bottom caps).
        """
        radius = geometry.radius
        length = geometry.length
        cap_area = np.pi * radius * radius
        diameter = radius * 2

        local_faces = {
            'cap_top': {
                'offset': [0, 0, length/2], 'normal': [0, 0, 1],
                'u_axis': [1, 0, 0], 'v_axis': [0, 1, 0], 'dims': [diameter, diameter]
            },
            'cap_bottom': {
                'offset': [0, 0, -length/2], 'normal': [0, 0, -1],
                'u_axis': [1, 0, 0], 'v_axis': [0, -1, 0], 'dims': [diameter, diameter]
            },
        }

        surfaces = self._transform_faces_to_world(
            local_faces, world_transform, link_name, collision_idx
        )

        # Mark as circular and override area calculation
        for surface in surfaces:
            surface.area = cap_area
            surface.is_circular = True

        return surfaces

    def _transform_faces_to_world(
        self,
        local_faces: dict,
        world_transform: NDArray,
        link_name: str,
        collision_idx: int
    ) -> List[Surface]:
        """Transform local face data to world frame surfaces.

        Args:
            local_faces: Dict of face data with offsets and normals.
            world_transform: 4x4 transformation matrix.
            link_name: Name of the parent link.
            collision_idx: Index of collision within link.

        Returns:
            List of Surface objects in world frame.
        """
        surfaces = []
        rot_matrix = world_transform[:3, :3]

        for face_name, face in local_faces.items():
            local_center = np.array(face['offset'])
            local_center_homogeneous = np.append(local_center, 1.0)
            world_center = (world_transform @ local_center_homogeneous)[:3]

            world_normal = rot_matrix @ np.array(face['normal'])
            world_normal = world_normal / np.linalg.norm(world_normal)

            world_u = rot_matrix @ np.array(face['u_axis'])
            world_u = world_u / np.linalg.norm(world_u)

            world_v = rot_matrix @ np.array(face['v_axis'])
            world_v = world_v / np.linalg.norm(world_v)

            surface_id = f'{link_name}:{collision_idx}:{face_name}'

            #TODO: (@silanus23) make this use surface analyzer
            # Calculate 4 corners for rectangular surfaces
            half_u = face['dims'][0] / 2.0
            half_v = face['dims'][1] / 2.0
            corners = [
                world_center + half_u * world_u + half_v * world_v,
                world_center + half_u * world_u - half_v * world_v,
                world_center - half_u * world_u - half_v * world_v,
                world_center - half_u * world_u + half_v * world_v,
            ]

            # Compute edges from corners
            edges = Surface.compute_boundary_edges(corners)

            # Create bounds dict
            bounds = {
                'u_min': -half_u,
                'u_max': half_u,
                'v_min': -half_v,
                'v_max': half_v
            }

            surfaces.append(Surface(
                surface_id=surface_id,
                center=world_center,
                normal=world_normal,
                corners=corners,
                edges=edges,
                u_axis=world_u,
                v_axis=world_v,
                bounds=bounds
            ))

        return surfaces
