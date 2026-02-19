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

Provides a domain-specific wrapper around LineSegment or ArcSegment for welding
applications, managing both geometric data and generated trajectory poses.
"""

from typing import Any, Dict, List, Optional

from .arc_segment import ArcSegment
from .line_segment import LineSegment


class Seam:
    """Represent a weld seam.

    Wraps a LineSegment or ArcSegment for geometry and holds weld-specific
    state (poses, normals, metadata).

    Attributes:
        segment: LineSegment or ArcSegment containing geometry
        poses: List of generated pose dictionaries, None if not generated yet
        is_generated: True if poses have been successfully generated
        config: Dictionary with metadata (is_edge_joint, normals, etc)
    """

    def __init__(
        self,
        seam_dict: Optional[Dict[str, List[float]]] = None,
        line_segment: Optional[LineSegment] = None,
        arc_segment: Optional[ArcSegment] = None,
        config: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Initialize seam from dict or segment object.

        @param seam_dict: Dictionary with 'start', 'end', optional
            'away_from_wall_vector'. Used for manual YAML workflow.
        @param line_segment: Pre-constructed LineSegment. Used for URDF
            auto-detection workflow (straight seams).
        @param arc_segment: Pre-constructed ArcSegment. Used for URDF
            auto-detection workflow (curved seams).
        @param config: Optional metadata dict (is_edge_joint, normals, etc)
        @throws ValueError: If no segment provided or multiple segments given
        @throws KeyError: If seam_dict missing required keys
        """
        # Validate that exactly one source is provided
        sources_provided = sum(
            [seam_dict is not None, line_segment is not None, arc_segment is not None]
        )

        if sources_provided == 0:
            raise ValueError(
                'Must provide one of: seam_dict, line_segment, or arc_segment'
            )
        if sources_provided > 1:
            raise ValueError('Cannot provide multiple segment sources')

        # Create segment from provided source
        if line_segment is not None:
            self.segment = line_segment
        elif arc_segment is not None:
            self.segment = arc_segment
        elif seam_dict is not None:
            if 'start' not in seam_dict:
                raise KeyError("seam_dict must contain 'start' key")
            if 'end' not in seam_dict:
                raise KeyError("seam_dict must contain 'end' key")

            away_vec = seam_dict.get('away_from_wall_vector', None)
            self.segment = LineSegment(
                seam_dict['start'], seam_dict['end'], away_from_wall_vector=away_vec
            )

        self.poses = None
        self.is_generated = False
        self.config = config or {}

    @property
    def line_segment(self) -> Optional[LineSegment]:
        """Get segment as LineSegment if it is one.

        @return LineSegment or None
        """
        if isinstance(self.segment, LineSegment):
            return self.segment
        return None

    @property
    def arc_segment(self) -> Optional[ArcSegment]:
        """Get segment as ArcSegment if it is one.

        @return ArcSegment or None
        """
        if isinstance(self.segment, ArcSegment):
            return self.segment
        return None

    @property
    def segment_type(self) -> str:
        """Get type of segment.

        @return 'line' or 'arc'
        """
        if isinstance(self.segment, LineSegment):
            return 'line'
        elif isinstance(self.segment, ArcSegment):
            return 'arc'
        return 'unknown'

    def length(self) -> float:
        """Get length of seam segment.

        @return Length in meters
        """
        return self.segment.length()

    def to_dict(self) -> Dict[str, Any]:
        """Convert seam to dictionary for JSON export."""
        if not self.is_generated:
            raise RuntimeError('Cannot export seam - poses not generated yet')

        result = {
            'segment_type': self.segment_type,
            'length_m': float(self.segment.length()),
            'poses': self.poses,
            'num_poses': len(self.poses) if self.poses else 0,
        }

        # Add segment-specific data
        if isinstance(self.segment, LineSegment):
            result['start'] = self.segment.start.tolist()
            result['end'] = self.segment.end.tolist()
            # Don't export away_from_wall_vector - it's computed per-point in WeldPlanner

        elif isinstance(self.segment, ArcSegment):
            result['start'] = self.segment.start.tolist()
            result['end'] = self.segment.end.tolist()
            result['center'] = self.segment.center.tolist()
            result['radius'] = float(self.segment.radius)
            # Don't export away_from_wall_vector - it's computed per-point in WeldPlanner

        # Include config metadata
        if self.config:
            result['config'] = {}
            for k, v in self.config.items():
                if k in ['normals_mesh_1', 'normals_mesh_2', 'smoothed_points']:
                    continue
                if hasattr(v, 'item'):
                    result['config'][k] = v.item()
                else:
                    result['config'][k] = v

        return result

    def __repr__(self) -> str:
        """Return string representation of Seam."""
        status = 'generated' if self.is_generated else 'not generated'

        has_away = (
            'with away_vector'
            if self.segment.away_from_wall_vector is not None
            else 'no away_vector'
        )

        edge_info = ''
        if self.config:
            is_edge = self.config.get('is_edge_joint', False)
            edge_info = f', edge_joint={is_edge}'

        return (
            f'Seam({self.segment_type}, length={self.segment.length():.3f}m, '
            f'{status}, {has_away}{edge_info})'
        )
