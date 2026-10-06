# Copyright 2025 Berkan Tali
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Seam - Weld seam with geometry and generated poses.

Provides a domain-specific wrapper around LineSegment, ArcSegment, or
PtPSegment for welding applications, managing both geometric data and
generated trajectory poses.
"""

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray

from .arc_segment import ArcSegment
from .line_segment import LineSegment
from .ptp_segment import PtPSegment


@dataclass
class SeamConfig:
    """Per-point seam data an extractor hands to WeldPlanner.

    Attributes:
        smoothed_points: Ordered seam points (N, 3), N >= 2.
        normals_main: Base surface normal per point (N, 3). A zero row means none was found;
            WeldPlanner fills it from a neighbour.
        normals_secondary: Wall normal per point (N, 3), zero rows as above.
        on_edge_1: Whether the seam follows a real edge of part 1.
        on_edge_2: Whether the seam follows a real edge of part 2.
    """

    smoothed_points: NDArray
    normals_main: NDArray
    normals_secondary: NDArray
    on_edge_1: bool
    on_edge_2: bool

    def __post_init__(self) -> None:
        """Coerce the arrays to float and check they describe the same N points.

        Raises:
            ValueError: If an array is not (N, 3), N < 2, or the three lengths differ.
        """
        for name in ('smoothed_points', 'normals_main', 'normals_secondary'):
            array = np.asarray(getattr(self, name), dtype=float)
            if array.ndim != 2 or array.shape[1] != 3:
                raise ValueError(f'{name} must be (N, 3), got shape {array.shape}')
            setattr(self, name, array)

        count = len(self.smoothed_points)
        if count < 2:
            raise ValueError(f'A seam needs at least 2 points, got {count}')
        for name in ('normals_main', 'normals_secondary'):
            if len(getattr(self, name)) != count:
                raise ValueError(
                    f'{name} has {len(getattr(self, name))} rows for {count} points')

        self.on_edge_1 = bool(self.on_edge_1)
        self.on_edge_2 = bool(self.on_edge_2)

    @property
    def is_edge_joint(self) -> bool:
        """Return True when both parts end on the seam (edge-to-edge)."""
        return self.on_edge_1 and self.on_edge_2


class Seam:
    """Weld seam with geometry and generated poses.

    Wraps LineSegment, ArcSegment, or PtPSegment with weld-specific metadata.

    Attributes:
        segment: LineSegment, ArcSegment, or PtPSegment containing geometry
        poses: List of generated pose dictionaries, None if not generated yet
        is_generated: True if poses have been successfully generated
        config: Points and normals for planning; None on a hand-written seam
    """

    def __init__(
        self,
        seam_dict: dict[str, list[float]] | None = None,
        line_segment: LineSegment | None = None,
        arc_segment: ArcSegment | None = None,
        ptp_segment: PtPSegment | None = None,
        config: SeamConfig | None = None,
    ) -> None:
        """Initialize seam from a seam dict, LineSegment, ArcSegment, or PtPSegment."""
        sources_provided = sum(
            [
                seam_dict is not None,
                line_segment is not None,
                arc_segment is not None,
                ptp_segment is not None,
            ]
        )

        if sources_provided == 0:
            raise ValueError(
                'Must provide one of: seam_dict, line_segment, arc_segment, or ptp_segment'
            )
        if sources_provided > 1:
            raise ValueError('Cannot provide multiple segment sources')

        if line_segment is not None:
            self.segment = line_segment
        elif arc_segment is not None:
            self.segment = arc_segment
        elif ptp_segment is not None:
            self.segment = ptp_segment
        elif seam_dict is not None:
            # Legacy dict format: construct LineSegment from start/end
            missing = [key for key in ('start', 'end') if key not in seam_dict]
            if missing:
                raise ValueError(f'seam_dict is missing key(s): {missing}')
            self.segment = LineSegment(seam_dict['start'], seam_dict['end'])

        self.poses = None
        self.is_generated = False
        self.config = config

    @property
    def line_segment(self) -> LineSegment | None:
        """Return segment as LineSegment if applicable, else None."""
        if isinstance(self.segment, LineSegment):
            return self.segment
        return None

    @property
    def arc_segment(self) -> ArcSegment | None:
        """Return segment as ArcSegment if applicable, else None."""
        if isinstance(self.segment, ArcSegment):
            return self.segment
        return None

    @property
    def ptp_segment(self) -> PtPSegment | None:
        """Return segment as PtPSegment if applicable, else None."""
        if isinstance(self.segment, PtPSegment):
            return self.segment
        return None

    @property
    def segment_type(self) -> str:
        """Return 'line', 'arc', 'ptp', or 'unknown'."""
        if isinstance(self.segment, LineSegment):
            return 'line'
        elif isinstance(self.segment, ArcSegment):
            return 'arc'
        elif isinstance(self.segment, PtPSegment):
            return 'ptp'
        return 'unknown'

    def length(self) -> float:
        """Return seam length in meters."""
        return self.segment.length()

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for JSON export.

        Returns:
            Dictionary with geometry and pose data

        Raises:
            RuntimeError: If poses not generated yet
        """
        if not self.is_generated:
            raise RuntimeError('Cannot export seam - poses not generated yet')

        result = {
            'segment_type': self.segment_type,
            'length_m': float(self.segment.length()),
            'poses': self.poses,
            'num_poses': len(self.poses) if self.poses else 0,
        }

        if isinstance(self.segment, LineSegment):
            result['start'] = self.segment.start.tolist()
            result['end'] = self.segment.end.tolist()

        elif isinstance(self.segment, ArcSegment):
            result['start'] = self.segment.start.tolist()
            result['end'] = self.segment.end.tolist()
            result['center'] = self.segment.center.tolist()
            result['radius'] = float(self.segment.radius)

        elif isinstance(self.segment, PtPSegment):
            result['start'] = self.segment.start.tolist()
            result['end'] = self.segment.end.tolist()
            result['num_points'] = len(self.segment.points)
            result['points'] = self.segment.points.tolist()

        if self.config is not None:
            result['config'] = {
                'is_edge_joint': self.config.is_edge_joint,
                'on_edge_1': self.config.on_edge_1,
                'on_edge_2': self.config.on_edge_2,
            }

        return result
