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

"""ArcSegment - Geometric primitive for circular arc paths.

Provides a reusable geometric primitive for representing circular arcs in 3D space.
Used for curved weld seams, particularly cylinder-to-surface joints.
"""

from typing import Any, Dict, List, Optional

import numpy as np
from numpy.typing import NDArray


class ArcSegment:
    """Represent a 3D circular arc segment.

    Defines an arc by center point, radius, and discretized points along the arc.
    Compatible with SeamExtractor output format.

    Attributes:
        center: Center point of circle as numpy array [x, y, z]
        radius: Radius of circle in meters
        points: Discretized points along arc (N, 3)
        away_from_wall_vector: Optional vector pointing away from material
    """

    def __init__(
        self,
        points: List[List[float]] | NDArray,
        center: List[float] | NDArray,
        radius: float,
        away_from_wall_vector: List[float] | NDArray | None = None,
    ) -> None:
        """Initialize arc segment from PathCreator geometry output.

        Args:
            points: Discretized points along arc (N x 3) - smoothed path
            center: Center point [x, y, z] from circle fit
            radius: Radius in meters from circle fit
            away_from_wall_vector: Optional vector [x, y, z] pointing away
        Raises:
            ValueError: If points are not 3D or radius is non-positive
        """
        self.points = np.array(points, dtype=float)
        self.center = np.array(center, dtype=float)

        if self.points.ndim != 2 or self.points.shape[1] != 3:
            raise ValueError('Points must be (N, 3) array')
        if len(self.points) < 2:
            raise ValueError('Need at least 2 points for arc')
        if self.center.shape != (3,):
            raise ValueError('Center must be 3D point [x, y, z]')
        if radius <= 0:
            raise ValueError(f'Radius must be positive, got {radius}')

        self.radius = float(radius)

        self.start = self.points[0]
        self.end = self.points[-1]

        if away_from_wall_vector is not None:
            away_from_wall_vector = np.array(away_from_wall_vector, dtype=float)
            if away_from_wall_vector.shape != (3,):
                raise ValueError('away_from_wall_vector must be 3D vector [x, y, z]')
            norm = np.linalg.norm(away_from_wall_vector)
            if norm > 1e-10:
                self.away_from_wall_vector = away_from_wall_vector / norm
            else:
                self.away_from_wall_vector = None
        else:
            self.away_from_wall_vector = None

    @classmethod
    def from_geometry_dict(
        cls, geometry: Dict[str, Any], away_from_wall_vector: Optional[NDArray] = None
    ) -> 'ArcSegment':
        """Create ArcSegment from PathCreator geometry output.

        Args:
            geometry: Geometry dict with 'type'='arc', 'points', 'center', 'radius'
            away_from_wall_vector: Optional away vector
        Returns:
            ArcSegment instance
        Raises:
            ValueError: If geometry type is not 'arc' or missing required fields
        """
        if geometry['type'] != 'arc':
            raise ValueError(f"Expected geometry type 'arc', got '{geometry['type']}'")

        return cls(
            points=geometry['points'],
            center=geometry['center'],
            radius=geometry['radius'],
            away_from_wall_vector=away_from_wall_vector,
        )

    def length(self) -> float:
        """Calculate arc length in meters."""
        if len(self.points) < 2:
            return 0.0

        distances = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        return float(np.sum(distances))

    def tangent_at(self, index: int) -> NDArray:
        """Calculate tangent vector at a discretized point.

        Raises:
            IndexError: If index out of bounds
            ValueError: If cannot compute tangent
        """
        if index < 0 or index >= len(self.points):
            raise IndexError(
                f'Index {index} out of bounds for {len(self.points)} points'
            )

        if index == 0:
            tangent = self.points[1] - self.points[0]
        elif index == len(self.points) - 1:
            tangent = self.points[-1] - self.points[-2]
        else:
            tangent = self.points[index + 1] - self.points[index - 1]

        norm = np.linalg.norm(tangent)
        if norm < 1e-10:
            raise ValueError(f'Cannot compute tangent at index {index}')

        return tangent / norm

    def midpoint(self) -> NDArray:
        """Get middle point of the arc."""
        mid_index = len(self.points) // 2
        return self.points[mid_index]

    def point_at(self, t: float) -> NDArray:
        """Get point along arc at parameter t."""
        if t <= 0:
            return self.points[0]
        if t >= 1:
            return self.points[-1]

        # Interpolate through discretized points
        float_index = t * (len(self.points) - 1)
        lower_index = int(np.floor(float_index))
        upper_index = min(lower_index + 1, len(self.points) - 1)

        alpha = float_index - lower_index
        return (1 - alpha) * self.points[lower_index] + alpha * self.points[upper_index]

    def tangent(self) -> NDArray:
        """Calculate normalized tangent vector at start of arc."""
        return self.tangent_at(0)

    def __repr__(self) -> str:
        """Return string representation of ArcSegment."""
        has_away = (
            'with away_vector'
            if self.away_from_wall_vector is not None
            else 'no away_vector'
        )
        return (
            f'ArcSegment(length={self.length():.3f}m, '
            f'radius={self.radius:.3f}m, {has_away})'
        )
