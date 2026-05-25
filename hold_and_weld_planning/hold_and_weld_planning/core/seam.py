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

"""Seam - Weld seam with geometry and generated poses.

Provides a domain-specific wrapper around LineSegment, ArcSegment, or
PtPSegment for welding applications, managing both geometric data and
generated trajectory poses.
"""

from typing import Any, Dict, List, Optional

from .arc_segment import ArcSegment
from .line_segment import LineSegment
from .ptp_segment import PtPSegment


class Seam:
    """Weld seam with geometry and generated poses.

    Wraps LineSegment, ArcSegment, or PtPSegment with weld-specific metadata.

    Attributes:
        segment: LineSegment, ArcSegment, or PtPSegment containing geometry
        poses: List of generated pose dictionaries, None if not generated yet
        is_generated: True if poses have been successfully generated
        config: Dictionary with metadata (is_edge_joint, normals, etc)
    """

    def __init__(
        self,
        seam_dict: Optional[Dict[str, List[float]]] = None,
        line_segment: Optional[LineSegment] = None,
        arc_segment: Optional[ArcSegment] = None,
        ptp_segment: Optional[PtPSegment] = None,
        config: Optional[Dict[str, Any]] = None,
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
            if 'start' not in seam_dict:
                raise KeyError("seam_dict must contain 'start' key")
            if 'end' not in seam_dict:
                raise KeyError("seam_dict must contain 'end' key")
            self.segment = LineSegment(seam_dict['start'], seam_dict['end'])

        self.poses = None
        self.is_generated = False
        self.config = config or {}

    @property
    def line_segment(self) -> Optional[LineSegment]:
        """Return segment as LineSegment if applicable, else None."""
        if isinstance(self.segment, LineSegment):
            return self.segment
        return None

    @property
    def arc_segment(self) -> Optional[ArcSegment]:
        """Return segment as ArcSegment if applicable, else None."""
        if isinstance(self.segment, ArcSegment):
            return self.segment
        return None

    @property
    def ptp_segment(self) -> Optional[PtPSegment]:
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

    def to_dict(self) -> Dict[str, Any]:
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

        if self.config:
            result['config'] = {}
            for k, v in self.config.items():
                if k in ['normals_mesh_1', 'normals_mesh_2', 'normals_main',
                         'normals_secondary', 'smoothed_points']:
                    continue

                if hasattr(v, 'item') and hasattr(v, 'shape') and v.shape == ():
                    result['config'][k] = v.item()
                elif hasattr(v, 'tolist'):
                    result['config'][k] = v.tolist()
                else:
                    result['config'][k] = v

        return result
