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

"""URDF seam planner - orchestrates URDF-based weld planning workflow.

This module coordinates the complete workflow for generating weld seam trajectories
from URDF geometry, including surface detection, joint type classification, and
pose generation.
"""

from typing import Any, Dict, List, Tuple

from .surface_processor import SurfaceProcessor
from .urdf_processor import URDFProcessor

from ..planning.weld_planner import WeldPlanner


class URDFSeamPlanner:
    """Orchestrate URDF-based weld seam generation."""

    def __init__(self, workpiece_config: Dict[str, Any]) -> None:
        """Initialize URDF seam planner with workpiece configuration.

        Args:
            workpiece_config: Dict with 'main_part' and 'secondary_part' containing:
                - urdf_path: Path to URDF/xacro file
                - world_pose: Dict with 'xyz' and 'rpy'
                - link_name: Name of the link to process
        """
        self.main_processor = URDFProcessor(
            workpiece_config['main_part']['urdf_path']
        )
        self.main_processor.set_world_pose(
            xyz=workpiece_config['main_part']['world_pose']['xyz'],
            rpy=workpiece_config['main_part']['world_pose']['rpy']
        )

        self.secondary_processor = URDFProcessor(
            workpiece_config['secondary_part']['urdf_path']
        )
        self.secondary_processor.set_world_pose(
            xyz=workpiece_config['secondary_part']['world_pose']['xyz'],
            rpy=workpiece_config['secondary_part']['world_pose']['rpy']
        )

        self.main_surface_processor = SurfaceProcessor(self.main_processor)
        self.secondary_surface_processor = SurfaceProcessor(self.secondary_processor)

        self.main_link = workpiece_config['main_part']['link_name']
        self.secondary_link = workpiece_config['secondary_part']['link_name']

        self.weld_planner = None

    def detect_joint_type_and_surface(
        self, seam: Any
    ) -> Tuple[str, Dict[str, List[float]]]:
        """Auto-detect joint type and surface information from geometry.

        Args:
            seam: Seam object with line_segment attribute.

        Returns:
            Tuple containing:
                - joint_type: 't_joint', 'corner_joint', or 'lap_joint'
                - surface_info: Dict with 'center' and 'normal' as lists

        Raises:
            ValueError: If no touching surfaces found between parts.
        """
        main_surfaces = self.main_surface_processor.extract_all_surfaces(
            self.main_link
        )
        secondary_surfaces = self.secondary_surface_processor.extract_all_surfaces(
            self.secondary_link
        )

        touching_pairs = self.main_surface_processor.find_touching_pairs(
            main_surfaces, secondary_surfaces
        )

        if not touching_pairs:
            raise ValueError(
                'No touching surfaces found between main and secondary parts'
            )
        main_surface, secondary_surface = (
            self.main_surface_processor.find_closest_pair_to_seam(
                touching_pairs, seam.line_segment
            )
        )

        joint_type = self.main_surface_processor.determine_joint_type(
            main_surface, secondary_surface
        )

        overlap_surface = self.main_surface_processor.get_overlap_surface(
            main_surface, secondary_surface
        )

        surface_info = {
            'center': overlap_surface['center'].tolist(),
            'normal': overlap_surface['normal'].tolist()
        }

        return joint_type, surface_info

    def generate_seams(
        self, seams: List[Any], parameters: Dict[str, Any]
    ) -> Tuple[str, Dict[str, List[float]], bool]:
        """Generate all seam poses with auto-detected configuration.

        Args:
            seams: List of Seam objects to generate poses for.
            parameters: Dict with weld parameters (joint_type will be added automatically).

        Returns:
            Tuple containing:
                - joint_type: Auto-detected joint type string
                - surface_info: Surface information dict used for generation
                - success: True if all seams generated successfully

        Raises:
            ValueError: If seams list is empty.
        """
        if not seams:
            raise ValueError('Seams list cannot be empty')

        joint_type, surface_info = self.detect_joint_type_and_surface(seams[0])
        parameters['joint_type'] = joint_type
        self.weld_planner = WeldPlanner(parameters)

        for seam in seams:
            success = self.weld_planner.generate_seam(seam, surface_info)
            if not success:
                return joint_type, surface_info, False

        return joint_type, surface_info, True
