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
                - auto_detect_seams: Optional bool to enable automatic seam detection
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

        self.auto_detect_seams = workpiece_config.get('auto_detect_seams', False)

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

    def prepare_surfaces_for_planner(
        self,
        main_surface: Dict[str, Any],
        secondary_surface: Dict[str, Any],
        joint_type: str,
        seam_lines: List[Any]
    ) -> Dict[str, Any]:
        """Prepare surface info for classical planner based on joint type.

        The classical planner has assumptions about which surface should be used
        as reference for calculating lean direction. This method ensures the 
        correct surface is selected based on joint type and seam positions.

        Args:
            main_surface: Main surface dict with face_id.
            secondary_surface: Secondary surface dict with face_id.
            joint_type: Detected joint type ('t_joint', 'butt_joint', etc.).
            seam_lines: List of LineSegment objects (seams).

        Returns:
            Surface info dict with 'center' and 'normal' for planner.

        Swap Logic:
            - T-joint: Use the surface that CONTAINS the seams (main if seams 
              inside main, secondary if seams inside secondary)
            - Butt joint: No swap, use overlap surface
            - Corner joint: Use the surface that contains non-edge seams
            - Lap joint: No swap for now, use overlap surface
        """
        # Ensure surfaces have corners for classification
        if 'corners' not in main_surface:
            main_surface['corners'] = self.main_surface_processor.get_surface_corners(
                main_surface
            )
        if 'corners' not in secondary_surface:
            secondary_surface['corners'] = self.main_surface_processor.get_surface_corners(
                secondary_surface
            )

        # Get overlap surface (used for most cases)
        overlap_surface = self.main_surface_processor.get_overlap_surface(
            main_surface, secondary_surface
        )

        if joint_type == 't_joint':
            # T-joint: Use the surface that contains the seams
            # Check which surface contains the seams
            positions = self.main_surface_processor.classify_seam_positions(
                seam_lines, main_surface, secondary_surface
            )
            
            all_inside_main = all(positions['inside_main'])
            all_inside_secondary = all(positions['inside_secondary'])
            
            if all_inside_main:
                # Seams inside main → use main surface
                return {
                    'center': main_surface['center'].tolist(),
                    'normal': main_surface['normal'].tolist()
                }
            elif all_inside_secondary:
                # Seams inside secondary → use secondary surface
                return {
                    'center': secondary_surface['center'].tolist(),
                    'normal': secondary_surface['normal'].tolist()
                }
            else:
                # Fallback to overlap
                return {
                    'center': overlap_surface['center'].tolist(),
                    'normal': overlap_surface['normal'].tolist()
                }

        elif joint_type == 'corner_joint':
            # Corner joint: Use the surface that contains non-edge seams
            positions = self.main_surface_processor.classify_seam_positions(
                seam_lines, main_surface, secondary_surface
            )
            
            num_inside_main = sum(positions['inside_main'])
            num_inside_secondary = sum(positions['inside_secondary'])
            
            if num_inside_main > 0 and num_inside_secondary == 0:
                # Non-edge seams in main → use main surface
                return {
                    'center': main_surface['center'].tolist(),
                    'normal': main_surface['normal'].tolist()
                }
            elif num_inside_secondary > 0 and num_inside_main == 0:
                # Non-edge seams in secondary → use secondary surface
                return {
                    'center': secondary_surface['center'].tolist(),
                    'normal': secondary_surface['normal'].tolist()
                }
            else:
                # Fallback to overlap
                return {
                    'center': overlap_surface['center'].tolist(),
                    'normal': overlap_surface['normal'].tolist()
                }

        elif joint_type == 'butt_joint':
            # Butt joint: No swap, use overlap surface
            return {
                'center': overlap_surface['center'].tolist(),
                'normal': overlap_surface['normal'].tolist()
            }

        elif joint_type == 'lap_joint':
            # Lap joint: Use away_from_wall_vector to determine which surface is "on top"
            # The "on top" surface (raised/overlapping) is the one we use for planning
            
            if not all_seams:
                # Fallback if no seams available
                return {
                    'center': overlap_surface['center'].tolist(),
                    'normal': overlap_surface['normal'].tolist()
                }
            
            # Get first seam's away_from_wall_vector
            first_seam = all_seams[0]
            away_vec = np.array(first_seam['away_from_wall_vector'])
            
            # Calculate seam midpoint
            seam_start = np.array(first_seam['start'])
            seam_end = np.array(first_seam['end'])
            seam_midpoint = (seam_start + seam_end) / 2.0
            
            # Vectors from seam to each surface center
            to_main = main_surface['center'] - seam_midpoint
            to_secondary = secondary_surface['center'] - seam_midpoint
            
            # away_from_wall_vector points toward the raised/overlapping part
            # The surface it points toward is the one we should use
            dot_main = np.dot(away_vec, to_main)
            dot_secondary = np.dot(away_vec, to_secondary)
            
            if dot_secondary > dot_main:
                # away_vec points more toward secondary → secondary is raised (on top)
                return {
                    'center': secondary_surface['center'].tolist(),
                    'normal': secondary_surface['normal'].tolist()
                }
            else:
                # away_vec points more toward main → main is raised (on top)
                return {
                    'center': main_surface['center'].tolist(),
                    'normal': main_surface['normal'].tolist()
                }

        else:
            # Unknown joint type, fallback to overlap
            return {
                'center': overlap_surface['center'].tolist(),
                'normal': overlap_surface['normal'].tolist()
            }

    def auto_detect_and_generate_seams(
        self, parameters: Dict[str, Any]
    ) -> Tuple[List[Dict[str, Any]], bool]:
        """Automatically detect all seams from geometry and generate poses.

        Args:
            parameters: Dict with weld parameters.

        Returns:
            Tuple containing:
                - seams_data: List of dicts, each with:
                    - seam_id: Unique identifier
                    - start: [x, y, z] start point
                    - end: [x, y, z] end point
                    - joint_type: Joint type string
                    - length: Seam length in meters
                    - poses: List of generated poses (if generation successful)
                - success: True if all seams generated successfully
        """
        # Auto-detect all seams from geometry
        detected_seams = self.main_surface_processor.auto_detect_all_seams(
            self.main_link,
            self.secondary_link,
            self.secondary_surface_processor
        )

        if not detected_seams:
            return [], False

        seams_data = []

        # Generate poses for each detected seam
        for idx, seam_dict in enumerate(detected_seams):
            seam_id = f'seam_{idx}'

            # Get main and secondary surfaces for this seam
            # Note: We need to fetch the actual surface dicts, not just IDs
            main_surfaces = self.main_surface_processor.extract_all_surfaces(
                self.main_link
            )
            secondary_surfaces = self.secondary_surface_processor.extract_all_surfaces(
                self.secondary_link
            )

            # Find the surfaces matching the IDs
            main_surface = None
            secondary_surface = None
            for surf in main_surfaces:
                if surf['face_id'] == seam_dict['main_surface_id']:
                    main_surface = surf
                    break
            for surf in secondary_surfaces:
                if surf['face_id'] == seam_dict['secondary_surface_id']:
                    secondary_surface = surf
                    break

            # Create a temporary seam object for this iteration
            from ..core.line_segment import LineSegment
            
            current_seam_line = LineSegment(
                seam_dict['start'],
                seam_dict['end']
            )

            if main_surface is None or secondary_surface is None:
                # Fallback to using surface_info from seam_dict
                surface_info = seam_dict['surface_info']
            else:
                # Prepare surfaces for planner based on joint type
                # Pass the current seam as a list
                surface_info = self.prepare_surfaces_for_planner(
                    main_surface,
                    secondary_surface,
                    seam_dict['joint_type'],
                    [current_seam_line]
                )

            # Set joint type in parameters
            parameters['joint_type'] = seam_dict['joint_type']

            # Initialize weld planner if needed
            if self.weld_planner is None:
                self.weld_planner = WeldPlanner(parameters)

            # Create a temporary seam object for pose generation
            from ..core.seam import Seam

            temp_seam = Seam({
                'start': seam_dict['start'],
                'end': seam_dict['end']
            })

            # Get away_from_wall_vector from seam_dict
            away_from_wall_vector = seam_dict.get('away_from_wall_vector')
            
            if away_from_wall_vector is None:
                raise ValueError(
                    f"Seam {seam_id} missing 'away_from_wall_vector'. "
                    "This should be calculated during auto-detection."
                )

            # Generate poses
            success = self.weld_planner.generate_seam(
                temp_seam, 
                surface_info,
                away_from_wall_vector=away_from_wall_vector
            )

            if success:
                seam_data = {
                    'seam_id': seam_id,
                    'start': seam_dict['start'],
                    'end': seam_dict['end'],
                    'joint_type': seam_dict['joint_type'],
                    'length': seam_dict['length'],
                    'main_surface_id': seam_dict['main_surface_id'],
                    'secondary_surface_id': seam_dict['secondary_surface_id'],
                    'poses': temp_seam.poses
                }
                seams_data.append(seam_data)
            else:
                # Seam detection succeeded but pose generation failed
                return seams_data, False

        return seams_data, True

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
            # For manual workflow, away_from_wall_vector must be provided
            # Users must add this field to their seam config
            # We pass None and let the planner raise an informative error
            success = self.weld_planner.generate_seam(
                seam, 
                surface_info,
                away_from_wall_vector=None  # Manual users must provide this
            )
            if not success:
                return joint_type, surface_info, False

        return joint_type, surface_info, True
