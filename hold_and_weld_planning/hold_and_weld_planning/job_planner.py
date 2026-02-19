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

"""JobPlanner - Orchestrate complete weld job planning pipeline.

This module coordinates mesh generation, seam extraction, and pose planning
to produce complete weld trajectories from URDF or STL inputs.
"""

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from .mesh.mesh_loader import MeshLoader
from .mesh.seam_extractor import SeamExtractor
from .mesh.shell_generator import ShellGenerator
from .planning.weld_planner import WeldPlanner
from .urdf.urdf_processor import URDFProcessor


class JobPlanner:
    """Orchestrate complete weld job planning from URDF or STL to trajectories.

    Coordinates the full pipeline:
    1. Load and process inputs (URDF or STL)
    2. Generate mesh shells
    3. Extract seam geometry
    4. Generate torch poses

    Supports URDF/URDF, STL/STL, and STL/URDF input combinations.
    This is the main entry point for programmatic use of the planning system.
    """

    def __init__(
        self,
        main_path: str,
        secondary_path: str,
        main_world_pose: Optional[Dict[str, list]] = None,
        secondary_world_pose: Optional[Dict[str, list]] = None,
        parameters: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Initialize job planner with input paths and poses.

            - work_angle_deg: Work angle in degrees
            - travel_angle_deg: Travel angle in degrees
            - gap_mm: Gap distance in millimeters
            - epsilon: Contact detection tolerance (default 0.01m)
            - num_smooth_points: Points per seam (default 100)
            - refine_iterations: Mesh subdivision iterations (default 32)

        Args:
            main_path: Path to main part URDF or STL (supports package://)
            secondary_path: Path to secondary part URDF or STL
            main_world_pose: Dict with 'xyz' and 'rpy' for main part pose
            secondary_world_pose: Dict with 'xyz' and 'rpy' for secondary
            parameters: Welding parameters dict with keys:
        Raises:
            ValueError: If required parameters are missing
        """
        self.main_path = main_path
        self.secondary_path = secondary_path
        self.main_world_transform = self._pose_to_matrix(main_world_pose)
        self.secondary_world_transform = self._pose_to_matrix(secondary_world_pose)

        self.parameters = parameters or {}
        self.parameters.setdefault('epsilon', 0.01)
        self.parameters.setdefault('num_smooth_points', 100)
        self.parameters.setdefault('refine_iterations', 32)

        required = ['work_angle_deg', 'travel_angle_deg', 'gap_mm']
        for param in required:
            if param not in self.parameters:
                raise ValueError(f"Missing required parameter: '{param}'")

        print(f'DEBUG job: starting with work_angle={self.parameters["work_angle_deg"]}deg, '
              f'gap={self.parameters["gap_mm"]}mm')

    def plan_job(self) -> list:
        """Execute complete planning pipeline.

        Returns:
            List of Seam objects with generated poses
        Raises:
            RuntimeError: If any pipeline stage fails
        """
        print('Generating mesh shells...')
        mesh_main, mesh_secondary = self._generate_shells()

        print('Extracting seams from geometry...')
        seam_extractor = SeamExtractor(mesh_main, mesh_secondary, self.parameters)
        seams = seam_extractor.extract_seams()

        if not seams:
            print('Warning: No seams detected')
            return []

        print(f'Detected {len(seams)} seam(s)')

        print('Generating weld poses...')
        weld_planner = WeldPlanner(self.parameters)

        for idx, seam in enumerate(seams):
            try:
                weld_planner.generate_seam(seam)
                num_poses = len(seam.poses) if seam.poses else 0
                print(f'  Seam {idx}: {num_poses} poses generated')
            except Exception as e:
                print(f'Warning: Failed to generate poses for seam {idx}: {e}')
                continue

        successful_seams = [s for s in seams if s.is_generated]
        print(f'Successfully planned {len(successful_seams)} seam(s)')

        return successful_seams

    def _generate_shells(self) -> tuple:
        """Generate trimesh shells for both parts, handling URDF and STL inputs.

        Returns:
            Tuple of (main_mesh, secondary_mesh) as trimesh objects
        Raises:
            RuntimeError: If shell generation fails
        """
        mesh_main = self._load_input(self.main_path, self.main_world_transform)
        mesh_secondary = self._load_input(
            self.secondary_path, self.secondary_world_transform
        )

        if not mesh_main.is_watertight:
            raise RuntimeError('Main mesh is not watertight')
        if not mesh_secondary.is_watertight:
            raise RuntimeError('Secondary mesh is not watertight')

        print(
            f'  Main mesh: {len(mesh_main.vertices)} vertices, '
            f'{len(mesh_main.faces)} faces'
        )
        print(
            f'  Secondary mesh: {len(mesh_secondary.vertices)} vertices, '
            f'{len(mesh_secondary.faces)} faces'
        )

        return mesh_main, mesh_secondary

    def _load_input(self, path: str, world_transform: np.ndarray) -> trimesh.Trimesh:
        """Load input as trimesh, detecting type by file extension.

        STL files are loaded via MeshLoader. URDF files are processed
        via URDFProcessor and ShellGenerator.

        Args:
            path: Path to URDF or STL file
            world_transform: 4x4 world pose matrix to apply
        Returns:
            Trimesh object ready for seam extraction
        Raises:
            RuntimeError: If loading or conversion fails
        """
        refine_iterations = self.parameters['refine_iterations']

        if Path(path).suffix.lower() == '.stl':
            loader = MeshLoader(
                mesh_path=path,
                world_transform=world_transform,
                refine_iterations=refine_iterations,
            )
            manifold_obj = loader.manifold
        else:
            urdf = URDFProcessor(path)
            urdf.world_transform = world_transform
            shell_gen = ShellGenerator(
                urdf.robot, world_transform, refine_iterations=refine_iterations
            )
            manifold_obj = shell_gen.create_shells_for_all_links()

        mesh_data = manifold_obj.to_mesh()

        return trimesh.Trimesh(
            vertices=mesh_data.vert_properties, faces=mesh_data.tri_verts
        )

    def _pose_to_matrix(self, pose: Optional[Dict[str, list]]) -> np.ndarray:
        """Convert xyz/rpy pose dict to 4x4 transform matrix.

        Args:
            pose: Dict with 'xyz' and 'rpy' keys, or None for identity
        Returns:
            4x4 homogeneous transform matrix
        """
        if pose is None:
            return np.eye(4)

        xyz = pose.get('xyz', [0.0, 0.0, 0.0])
        rpy = pose.get('rpy', [0.0, 0.0, 0.0])

        rot_matrix = Rotation.from_euler('xyz', rpy).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = xyz

        return T
