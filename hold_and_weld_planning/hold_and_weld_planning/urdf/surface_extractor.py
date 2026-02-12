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

"""Surface extractor - extracts planar surfaces from URDF collision geometries.

This module provides tools for extracting planar surfaces from URDF collision
geometries (boxes and cylinders). Surfaces are represented as dictionaries
containing center, normal, axes, bounds, and area information.
"""

from typing import Any, Dict, List

import numpy as np
from numpy.typing import NDArray


class SurfaceExtractor:
    """Extract planar surfaces from URDF collision geometries.
    
    Supports box and cylinder geometries. Returns surfaces as dictionaries
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
    ) -> List[Dict[str, Any]]:
        """Extract all planar surfaces from multiple links.

        Args:
            link_names: List of link names to process.

        Returns:
            List of surface dictionaries from all links.
        """
        all_surfaces = []

        for link_name in link_names:
            surfaces = self.extract_all_surfaces(link_name)
            all_surfaces.extend(surfaces)

        return all_surfaces

    def extract_all_surfaces(self, link_name: str) -> List[Dict[str, Any]]:
        """Extract all planar surfaces from a link's collision geometries.

        Args:
            link_name: Name of the link to analyze.

        Returns:
            List of surface dictionaries, each containing:
                - center: np.array [x, y, z] in world frame
                - normal: np.array [x, y, z] unit vector in world frame
                - u_axis: np.array [x, y, z] first bound direction
                - v_axis: np.array [x, y, z] second bound direction
                - area: surface area in m²
                - bounds: [dim1, dim2] dimensions along u_axis and v_axis
                - face_id: identifier string (e.g., 'link:0:top')
                - is_circular: bool indicating if surface is circular (cylinder cap)

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
    ) -> List[Dict[str, Any]]:
        """Extract 6 planar surfaces from a box geometry.

        Args:
            geometry: Box geometry with 'size' attribute.
            world_transform: 4x4 transformation matrix to world frame.
            link_name: Name of the parent link.
            collision_idx: Index of collision within link.

        Returns:
            List of 6 surface dictionaries.
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
    ) -> List[Dict[str, Any]]:
        """Extract 2 planar cap surfaces from a cylinder geometry.

        Args:
            geometry: Cylinder geometry with radius and length.
            world_transform: 4x4 transformation matrix to world frame.
            link_name: Name of the parent link.
            collision_idx: Index of collision within link.

        Returns:
            List of 2 surface dictionaries (top and bottom caps).
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
            surface['area'] = cap_area
            surface['is_circular'] = True

        return surfaces

    def _transform_faces_to_world(
        self,
        local_faces: Dict[str, Dict[str, Any]],
        world_transform: NDArray,
        link_name: str,
        collision_idx: int
    ) -> List[Dict[str, Any]]:
        """Transform local face data to world frame surfaces.

        Args:
            local_faces: Dict of face data with offsets and normals.
            world_transform: 4x4 transformation matrix.
            link_name: Name of the parent link.
            collision_idx: Index of collision within link.

        Returns:
            List of surface dictionaries in world frame.
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

            area = face['dims'][0] * face['dims'][1]
            face_id = f'{link_name}:{collision_idx}:{face_name}'

            surfaces.append({
                'center': world_center,
                'normal': world_normal,
                'u_axis': world_u,
                'v_axis': world_v,
                'area': area,
                'bounds': face['dims'],
                'face_id': face_id,
                'is_circular': False
            })
        return surfaces
