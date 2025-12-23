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

"""Seam - Weld seam with geometry and generated poses."""

from .line_segment import LineSegment


class Seam:
    """
    Represent a weld seam.

    Wraps a LineSegment for geometry and holds weld-specific state (poses).
    """

    def __init__(self, seam_dict):
        """
        Initialize seam from YAML dictionary.

        Args:
            seam_dict : dict
                Dictionary with 'start' and 'end' keys

        """
        self.line_segment = LineSegment(seam_dict['start'], seam_dict['end'])
        self.poses = None
        self.is_generated = False

    def to_dict(self):
        """
        Convert seam to dictionary for JSON export.

        Returns
        -------
        dict
            Dictionary with seam data and generated poses

        Raises
        ------
        RuntimeError
            If poses not generated yet

        """
        if not self.is_generated:
            raise RuntimeError("Cannot export seam - poses not generated yet")

        return {
            'start': self.line_segment.start.tolist(),
            'end': self.line_segment.end.tolist(),
            'length_m': self.line_segment.length(),
            'poses': self.poses,
            'num_poses': len(self.poses) if self.poses else 0
        }

    def __repr__(self):
        """Return string representation of seam."""
        status = "generated" if self.is_generated else "not generated"
        return f"Seam(length={self.line_segment.length():.3f}m, {status})"
