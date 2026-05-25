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

"""PtPSegment - Point-to-point segment for complex curves.

Provides a geometric primitive for curves that cannot be adequately
represented as a line or arc. Analogous to Pilz PTP motion mode —
the robot moves through each point in sequence with no geometric
interpolation between them.

Used when sliding window classification finds a curve that fits neither
a line nor an arc within acceptable error thresholds.
"""

from typing import List

import numpy as np
from numpy.typing import NDArray


class PtPSegment:
    """Point-to-point segment defined by an ordered array of 3D points.

    Carries the raw point sequence for direct waypoint generation.
    Tangents are computed via finite differences on the point array.

    Attributes:
        points: Ordered points along curve as (N, 3) numpy array
        start:  First point of segment
        end:    Last point of segment
    """

    def __init__(
        self,
        points: List[List[float]] | NDArray,
    ) -> None:
        """Initialize PtP segment with ordered point array.

        Args:
            points: Ordered 3D points along curve. Must be (N, 3) with N >= 2.

        Raises:
            ValueError: If points array is invalid or has fewer than 2 points.
        """
        self.points = np.array(points, dtype=float)

        if self.points.ndim != 2 or self.points.shape[1] != 3:
            raise ValueError(
                f'Points must be (N, 3) array, got shape {self.points.shape}'
            )
        if len(self.points) < 2:
            raise ValueError(
                f'Need at least 2 points for PtPSegment, got {len(self.points)}'
            )

        self.start = self.points[0]
        self.end = self.points[-1]

    def length(self) -> float:
        """Return total arc length in meters."""
        distances = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        return float(np.sum(distances))

    def tangent(self) -> NDArray:
        """Return normalized tangent vector at start of segment."""
        return self.tangent_at(0)

    def tangent_at(self, index: int) -> NDArray:
        """Return normalized tangent vector at point index via finite differences.

        Uses forward difference at start, backward difference at end,
        central difference at interior points.

        Args:
            index: Point index in [0, N-1]

        Returns:
            Normalized tangent vector (3,)

        Raises:
            IndexError: If index is out of bounds.
            ValueError: If tangent is degenerate (zero length) at this index.
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
            raise ValueError(
                f'Degenerate tangent at index {index} — consecutive points are coincident'
            )

        return tangent / norm

    def midpoint(self) -> NDArray:
        """Return point closest to arc-length midpoint of segment."""
        target = self.length() / 2.0
        cumulative = 0.0

        for i in range(1, len(self.points)):
            step = np.linalg.norm(self.points[i] - self.points[i - 1])
            if cumulative + step >= target:
                # Interpolate within this segment
                remaining = target - cumulative
                t = remaining / step if step > 1e-10 else 0.0
                return self.points[i - 1] + t * (self.points[i] - self.points[i - 1])
            cumulative += step

        return self.points[len(self.points) // 2]

    def point_at(self, t: float) -> NDArray:
        """Return point at arc-length parameter t (0=start, 1=end).

        Interpolates along the point array by normalized arc length.
        t is clamped to [0, 1].

        Args:
            t: Arc-length parameter in [0, 1]

        Returns:
            Interpolated point (3,)
        """
        t = float(np.clip(t, 0.0, 1.0))

        if t <= 0.0:
            return self.points[0].copy()
        if t >= 1.0:
            return self.points[-1].copy()

        total_length = self.length()
        if total_length < 1e-10:
            return self.points[0].copy()

        target = t * total_length
        cumulative = 0.0

        for i in range(1, len(self.points)):
            step = np.linalg.norm(self.points[i] - self.points[i - 1])
            if cumulative + step >= target:
                remaining = target - cumulative
                alpha = remaining / step if step > 1e-10 else 0.0
                return self.points[i - 1] + alpha * (self.points[i] - self.points[i - 1])
            cumulative += step

        return self.points[-1].copy()

    def __repr__(self) -> str:
        """Return string representation of PtPSegment."""
        return (
            f'PtPSegment(length={self.length():.3f}m, '
            f'points={len(self.points)})'
        )
