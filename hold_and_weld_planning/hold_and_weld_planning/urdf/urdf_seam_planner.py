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

"""URDF seam planner - orchestrate seam detection and pose generation."""

from typing import Any, Dict, List, Optional

from .seam_detector import SeamDetector
from .surface_analyzer import SurfaceAnalyzer
from .surface_extractor import SurfaceExtractor
from .urdf_processor import URDFProcessor
from ..core.line_segment import LineSegment
from ..core.seam import Seam
from ..planning.weld_planner import WeldPlanner

class URDFSeamPlanner:
    """High-level planner for URDF-based welding (geometry-driven)."""

    def __init__(
        self,
        main_urdf_path: str,
        secondary_urdf_path: str,  # Required, not Optional
        main_world_pose: Optional[Dict[str, List[float]]] = None,
        secondary_world_pose: Optional[Dict[str, List[float]]] = None
    ) -> None:
        """Initialize URDF seam planner.

        Args:
            main_urdf_path: Path to main part URDF.
            secondary_urdf_path: Path to secondary part URDF.
            main_world_pose: Dict with 'xyz' and 'rpy' for main part.
            secondary_world_pose: Dict with 'xyz' and 'rpy' for secondary part.
        """
        # Create main processor
        main_processor = URDFProcessor(main_urdf_path)
        if main_world_pose:
            main_processor.set_world_pose(
                main_world_pose['xyz'], main_world_pose['rpy']
            )
        self.main_extractor = SurfaceExtractor(main_processor)

        # Create secondary processor (always separate, even if same URDF)
        secondary_processor = URDFProcessor(secondary_urdf_path)
        if secondary_world_pose:
            secondary_processor.set_world_pose(
                secondary_world_pose['xyz'], secondary_world_pose['rpy']
            )
        self.secondary_extractor = SurfaceExtractor(secondary_processor)

        self.analyzer = SurfaceAnalyzer()
        self.seam_detector = SeamDetector(self.main_extractor, self.analyzer)
        self.weld_planner = None

    def auto_detect_and_generate_seams(
        self,
        main_link: str,
        secondary_link: str,
        parameters: Dict[str, Any],
        min_seam_length_m: float = 0.01
    ) -> List[Seam]:
        """Auto-detect seams and generate poses from URDF geometry.

        Args:
            main_link: Link name in main part.
            secondary_link: Link name in secondary part.
            parameters: Welding parameters dict.
            min_seam_length_m: Minimum seam length (default 10mm).

        Returns:
            List of Seam objects with generated poses.
        """
        seam_data_list = self.seam_detector.auto_detect_all_seams(
            main_link,
            secondary_link,
            self.secondary_extractor,
            min_seam_length_m
        )

        if not seam_data_list:
            return []

        seams = []
        for seam_data in seam_data_list:
            line_segment = LineSegment(
                start=seam_data['start'],
                end=seam_data['end'],
                away_from_wall_vector=seam_data['away_from_wall_vector']
            )

            seam = Seam(
                line_segment=line_segment,
                config={
                    'is_edge_joint': seam_data['is_edge_joint'],
                    'on_surface': seam_data['on_surface']
                }
            )

            weld_planner = WeldPlanner(parameters)
            surface_info = seam_data['reference_surface']
            weld_planner.generate_seam(seam, surface_info)

            seams.append(seam)

        return seams
