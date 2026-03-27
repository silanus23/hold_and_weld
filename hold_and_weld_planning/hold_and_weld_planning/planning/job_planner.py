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

"""Orchestrate complete weld job planning from URDF/CAD to trajectories."""

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from ..mesh.mesh_loader import MeshLoader
from ..mesh.seam_extractor import SeamExtractor
from ..mesh.shell_generator import ShellGenerator

from ..occt.occt_generator import OCCTGenerator
from ..occt.occt_loader import OCCTLoader
from ..occt.seam_extractor_occt import SeamExtractorOCCT

from .weld_planner import WeldPlanner
from ..urdf.urdf_processor import URDFProcessor


class JobPlanner:
    """Orchestrate complete weld job planning from URDF/CAD to trajectories.

    Main entry point for programmatic use of the planning system.
    Supports URDF, STL, and STEP inputs with automatic mode detection.
    """

    def __init__(
        self,
        main_path: str,
        secondary_path: str,
        main_world_pose: Optional[Dict[str, list]] = None,
        secondary_world_pose: Optional[Dict[str, list]] = None,
        parameters: Optional[Dict[str, Any]] = None,
        mode: str = 'auto',
    ) -> None:
        """Initialize job planner.

        Args:
            main_path: Path to main part (URDF/STL/STEP, supports package://)
            secondary_path: Path to secondary part
            main_world_pose: Dict with 'xyz' and 'rpy' for main part
            secondary_world_pose: Dict with 'xyz' and 'rpy' for secondary
            parameters: Dict with work_angle_deg, travel_angle_deg, gap_mm,
                       epsilon (optional), num_smooth_points (optional),
                       refine_iterations (optional)
            mode: 'auto', 'mesh', or 'occt'

        Raises:
            ValueError: If parameters missing or mode invalid
        """
        self.main_path = main_path
        self.secondary_path = secondary_path
        self.main_world_transform = self._pose_to_matrix(main_world_pose)
        self.secondary_world_transform = self._pose_to_matrix(secondary_world_pose)

        self.parameters = parameters or {}

        if mode not in ['auto', 'mesh', 'occt']:
            raise ValueError(f"Mode must be 'auto', 'mesh', or 'occt', got '{mode}'")

        if mode == 'auto':
            self.mode = self._detect_mode(main_path, secondary_path)
        else:
            self.mode = mode

        if self.mode == 'occt':
            self.parameters.setdefault('epsilon', 1e-3)
        else:
            self.parameters.setdefault('epsilon', 0.01)

        self.parameters.setdefault('num_smooth_points', 100)
        self.parameters.setdefault('refine_iterations', 32)

        required = ['work_angle_deg', 'travel_angle_deg', 'gap_mm']
        for param in required:
            if param not in self.parameters:
                raise ValueError(f"Missing required parameter: '{param}'")

        print(f'JobPlanner initialized in {self.mode.upper()} mode')
        print(f'  work_angle={self.parameters["work_angle_deg"]}deg, '
              f'gap={self.parameters["gap_mm"]}mm, '
              f'tolerance={self.parameters["epsilon"]*1000}mm')

    def _detect_mode(self, main_path: str, secondary_path: str) -> str:
        """Auto-detect processing mode from file extensions."""
        main_ext = Path(main_path).suffix.lower()
        secondary_ext = Path(secondary_path).suffix.lower()

        occt_extensions = {'.step', '.stp', '.iges', '.igs'}
        mesh_extensions = {'.stl', '.urdf', '.xacro'}

        if main_ext in occt_extensions or secondary_ext in occt_extensions:
            return 'occt'

        if main_ext in mesh_extensions and secondary_ext in mesh_extensions:
            return 'mesh'

        return 'mesh'

    def plan_job(self) -> list:
        """Execute complete planning pipeline.

        Returns:
            List of Seam objects with generated poses

        Raises:
            RuntimeError: If pipeline stage fails
        """
        if self.mode == 'occt':
            return self._plan_job_occt()
        else:
            return self._plan_job_mesh()

    def _plan_job_mesh(self) -> list:
        """Execute mesh-based planning pipeline using manifold3d and CGAL."""
        print('Generating mesh shells (manifold3d)...')
        mesh_main, mesh_secondary = self._generate_shells()

        print(f'Mesh 1: watertight={mesh_main.is_watertight}, '
              f'faces={len(mesh_main.faces)}, bounds={mesh_main.bounds}')
        print(f'Mesh 2: watertight={mesh_secondary.is_watertight}, '
              f'faces={len(mesh_secondary.faces)}, bounds={mesh_secondary.bounds}')

        print('Extracting seams from geometry (CGAL)...')
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

    def _plan_job_occt(self) -> list:
        """Execute OCCT-based planning pipeline using pythonocc-core."""
        print('Generating OCCT shapes (pythonocc-core)...')
        shape_main, shape_secondary = self._generate_occt_shapes()

        print('Extracting seams from geometry (OCCT)...')
        seam_extractor = SeamExtractorOCCT(shape_main, shape_secondary, self.parameters)
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
        """Generate trimesh shells for both parts in mesh mode."""
        mesh_main = self._load_input_mesh(self.main_path, self.main_world_transform)
        mesh_secondary = self._load_input_mesh(
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

    def _generate_occt_shapes(self) -> tuple:
        """Generate OCCT TopoDS_Shape objects for both parts in OCCT mode."""
        shape_main = self._load_input_occt(self.main_path, self.main_world_transform)
        shape_secondary = self._load_input_occt(
            self.secondary_path, self.secondary_world_transform
        )

        print('  Main shape: OCCT TopoDS_Shape')
        print('  Secondary shape: OCCT TopoDS_Shape')

        return shape_main, shape_secondary

    def _load_input_mesh(self, path: str, world_transform: np.ndarray) -> trimesh.Trimesh:
        """Load URDF or STL as trimesh with world transform applied."""
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

    def _load_input_occt(self, path: str, world_transform: np.ndarray):
        """Load URDF, STEP, or IGES as OCCT shape with world transform applied."""
        path_suffix = Path(path).suffix.lower()

        if path_suffix in ['.step', '.stp', '.iges', '.igs']:
            loader = OCCTLoader(
                cad_path=path,
                world_transform=world_transform,
            )
            return loader.shape
        else:
            urdf = URDFProcessor(path)
            urdf.world_transform = world_transform
            occt_gen = OCCTGenerator(urdf.robot, world_transform)
            return occt_gen.create_shape_for_all_links()

    def _pose_to_matrix(self, pose: Optional[Dict[str, list]]) -> np.ndarray:
        """Convert xyz/rpy dict to 4x4 transformation matrix."""
        if pose is None:
            return np.eye(4)

        xyz = pose.get('xyz', [0.0, 0.0, 0.0])
        rpy = pose.get('rpy', [0.0, 0.0, 0.0])

        rot_matrix = Rotation.from_euler('xyz', rpy).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = xyz

        return T
