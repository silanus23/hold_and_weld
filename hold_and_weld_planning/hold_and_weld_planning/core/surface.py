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

"""Surface - Geometric representation of workpiece surfaces.

Provides a structured dataclass for representing surface geometry including
corners, edges, normals, and local coordinate frames. Supports N-sided polygons
and future extension to cylindrical surfaces.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray


@dataclass
class Surface:
    """Represent a workpiece surface with geometry and coordinate frame.

    Attributes:
        surface_id: Unique identifier (e.g., 'base_link:0:top')
        center: Center point [x, y, z] in world coordinates
        normal: Surface normal vector [nx, ny, nz] (unit vector)
        corners: List of corner points defining the surface boundary (N corners)
        edges: List of boundary edges as (start, end) point pairs
        u_axis: First basis vector in surface plane (unit vector)
        v_axis: Second basis vector in surface plane (unit vector)
        bounds: Surface bounds in local UV coordinates
    """

    surface_id: str
    center: NDArray
    normal: NDArray
    corners: List[NDArray]
    edges: List[Tuple[NDArray, NDArray]]
    u_axis: NDArray
    v_axis: NDArray
    bounds: Dict[str, float]

    @staticmethod
    def compute_boundary_edges(corners: List[NDArray]) -> List[Tuple[NDArray, NDArray]]:
        """Compute boundary edges from ordered corner points.

        Args:
            corners: List of N corner points in order.

        Returns:
            List of N edges as (start, end) tuples.

        Example:
            For 4 corners [A, B, C, D], returns:
            [(A, B), (B, C), (C, D), (D, A)]
        """
        if len(corners) < 3:
            raise ValueError(
                f'Need at least 3 corners to form edges, got {len(corners)}'
            )

        edges = []
        n = len(corners)
        for i in range(n):
            start = corners[i]
            end = corners[(i + 1) % n]  # Wrap around to first corner
            edges.append((start, end))

        return edges

    @classmethod
    def from_dict(cls, surface_dict: Dict[str, Any]) -> 'Surface':
        """Create Surface from dictionary representation.

        Args:
            surface_dict: Dictionary containing surface data.

        Returns:
            Surface instance with computed edges if not provided.

        Raises:
            KeyError: If required keys are missing.
            ValueError: If corner count is invalid.
        """
        # Convert lists to numpy arrays
        center = np.array(surface_dict['center'], dtype=float)
        normal = np.array(surface_dict['normal'], dtype=float)
        corners = [np.array(c, dtype=float) for c in surface_dict['corners']]
        u_axis = np.array(surface_dict['u_axis'], dtype=float)
        v_axis = np.array(surface_dict['v_axis'], dtype=float)

        # Compute edges if not provided
        if 'edges' in surface_dict and surface_dict['edges']:
            edges = [
                (np.array(edge[0], dtype=float), np.array(edge[1], dtype=float))
                for edge in surface_dict['edges']
            ]
        else:
            edges = cls.compute_boundary_edges(corners)

        surface_id = surface_dict.get('surface_id', surface_dict.get('id', ''))
        return cls(
            surface_id=surface_id,
            center=center,
            normal=normal,
            corners=corners,
            edges=edges,
            u_axis=u_axis,
            v_axis=v_axis,
            bounds=surface_dict['bounds'],
        )

    def to_dict(self) -> Dict[str, Any]:
        """Convert Surface to dictionary representation.

        Returns:
            Dictionary with all surface data, with numpy arrays converted to
            lists. Compatible with JSON serialization and classical planner.
        """
        return {
            'id': self.surface_id,
            'center': self.center.tolist(),
            'normal': self.normal.tolist(),
            'corners': [c.tolist() for c in self.corners],
            'edges': [(e[0].tolist(), e[1].tolist()) for e in self.edges],
            'u_axis': self.u_axis.tolist(),
            'v_axis': self.v_axis.tolist(),
            'bounds': self.bounds,
        }

    def get_surface_info(self) -> Dict[str, List[float]]:
        """Get surface info dict for classical planner interface.

        Returns:
            Dictionary with 'center' and 'normal' keys for weld_planner.
        """
        return {'center': self.center.tolist(), 'normal': self.normal.tolist()}

    def num_corners(self) -> int:
        """Get number of corners defining this surface.

        Returns:
            Number of corners (N >= 3).
        """
        return len(self.corners)

    def num_edges(self) -> int:
        """Get number of boundary edges.

        Returns:
            Number of edges (equal to number of corners).
        """
        return len(self.edges)

    def __repr__(self) -> str:
        """Return string representation of Surface.

        Returns:
            String showing surface ID and corner count.
        """
        return f"Surface(id='{self.surface_id}', corners={self.num_corners()})"
