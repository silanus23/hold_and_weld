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

"""LineSegment - Pure geometry primitive for 3D line segments.

Provides a reusable geometric primitive for representing line segments in 3D space.
Useful for mesh edges, rays, seam paths, and other linear geometric entities.
"""

from typing import List

import numpy as np
from numpy.typing import NDArray


class LineSegment:
    """Represent a 3D line segment defined by start and end points.

    Pure geometry class - no application-specific logic.
    Reusable for mesh edges, rays, seams, etc.
    """

    def __init__(self, start: List[float] | NDArray, end: List[float] | NDArray) -> None:
        """Initialize line segment from start and end points.

        Args:
            start: Start point [x, y, z] as list or numpy array.
            end: End point [x, y, z] as list or numpy array.

        Raises:
            ValueError: If points are not 3D.
        """
        self.start = np.array(start, dtype=float)
        self.end = np.array(end, dtype=float)

        if self.start.shape != (3,) or self.end.shape != (3,):
            raise ValueError('Start and end must be 3D points [x, y, z]')

    def length(self) -> float:
        """Calculate segment length in meters.

        Returns:
            Length of the segment as a float.
        """
        return float(np.linalg.norm(self.end - self.start))

    def tangent(self) -> NDArray:
        """Calculate normalized tangent vector along the segment.

        Returns:
            Normalized direction vector from start to end.

        Raises:
            ValueError: If segment is degenerate (zero length).
        """
        length = self.length()
        if length < 1e-9:
            raise ValueError('Segment is degenerate (zero length)')
        return (self.end - self.start) / length

    def midpoint(self) -> NDArray:
        """Get middle point of the segment.

        Returns:
            Point at the exact middle between start and end.
        """
        return (self.start + self.end) / 2.0

    def point_at(self, t: float) -> NDArray:
        """Get point along segment at parameter t.

        Args:
            t: Parameter value (0 = start, 1 = end).

        Returns:
            Point at parameter t (linear interpolation).
        """
        return self.start + t * (self.end - self.start)

    def __repr__(self) -> str:
        """Return string representation of LineSegment.

        Returns:
            String showing segment length.
        """
        return f'LineSegment(length={self.length():.3f}m)'
