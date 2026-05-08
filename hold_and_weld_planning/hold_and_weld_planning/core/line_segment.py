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
    """3D line segment defined by start and end points.

    Attributes:
        start: Start point as numpy array [x, y, z]
        end: End point as numpy array [x, y, z]
    """

    def __init__(
        self,
        start: List[float] | NDArray,
        end: List[float] | NDArray,
    ) -> None:
        """Initialize line segment with start and end points."""
        self.start = np.array(start, dtype=float)
        self.end = np.array(end, dtype=float)

        if self.start.shape != (3,) or self.end.shape != (3,):
            raise ValueError('Start and end must be 3D points [x, y, z]')

    def length(self) -> float:
        """Return segment length in meters."""
        return float(np.linalg.norm(self.end - self.start))

    def tangent(self) -> NDArray:
        """Return normalized tangent vector along segment.

        Raises:
            ValueError: If segment has zero length.
        """
        length = self.length()
        if length < 1e-9:
            raise ValueError('Segment is degenerate (zero length)')
        return (self.end - self.start) / length

    def midpoint(self) -> NDArray:
        """Return middle point of segment."""
        return (self.start + self.end) / 2.0

    def point_at(self, t: float) -> NDArray:
        """Return point at parameter t (0=start, 1=end).

        Note: t is not clamped — values outside [0, 1] extrapolate beyond the segment.
        """
        return self.start + t * (self.end - self.start)
