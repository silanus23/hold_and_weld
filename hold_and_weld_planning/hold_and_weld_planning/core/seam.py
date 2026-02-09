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

Provides a domain-specific wrapper around LineSegment for welding applications,
managing both geometric data and generated trajectory poses.
"""

from typing import Any, Dict, List

from .line_segment import LineSegment


class Seam:
    """Represent a weld seam.

    Wraps a LineSegment for geometry and holds weld-specific state (poses).
    
    Attributes:
        line_segment: LineSegment containing start, end, and away_from_wall_vector.
        poses: List of generated pose dictionaries, None if not generated yet.
        is_generated: True if poses have been successfully generated.
    """

    def __init__(self, seam_dict: Dict[str, List[float]]) -> None:
        """Initialize seam from YAML/config dictionary.

        Args:
            seam_dict: Dictionary with required keys:
                - 'start': [x, y, z] coordinates
                - 'end': [x, y, z] coordinates
                Optional keys:
                - 'away_from_wall_vector': [x, y, z] vector (for manual workflow)

        Raises:
            KeyError: If 'start' or 'end' missing from seam_dict.
        """
        if 'start' not in seam_dict:
            raise KeyError("seam_dict must contain 'start' key")
        if 'end' not in seam_dict:
            raise KeyError("seam_dict must contain 'end' key")

        # Extract away_from_wall_vector if present
        away_vec = seam_dict.get('away_from_wall_vector', None)
        
        self.line_segment = LineSegment(
            seam_dict['start'], 
            seam_dict['end'],
            away_from_wall_vector=away_vec
        )
        self.poses = None
        self.is_generated = False

    def to_dict(self) -> Dict[str, Any]:
        """Convert seam to dictionary for JSON export.

        Returns:
            Dictionary with 'start', 'end', 'length_m', 'away_from_wall_vector', 
            'poses', and 'num_poses' keys.

        Raises:
            RuntimeError: If poses not generated yet.
        """
        if not self.is_generated:
            raise RuntimeError('Cannot export seam - poses not generated yet')

        result = {
            'start': self.line_segment.start.tolist(),
            'end': self.line_segment.end.tolist(),
            'length_m': self.line_segment.length(),
            'poses': self.poses,
            'num_poses': len(self.poses) if self.poses else 0
        }

        # Include away_from_wall_vector for debugging if it's set
        if self.line_segment.away_from_wall_vector is not None:
            result['away_from_wall_vector'] = self.line_segment.away_from_wall_vector.tolist()

        return result

    def __repr__(self) -> str:
        """Return string representation of Seam.

        Returns:
            String showing length, generation status, and away_vector status.
        """
        status = 'generated' if self.is_generated else 'not generated'
        has_away = 'with away_vector' if self.line_segment.away_from_wall_vector is not None else 'no away_vector'
        return f'Seam(length={self.line_segment.length():.3f}m, {status}, {has_away})'
