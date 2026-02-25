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

This module coordinates geometry generation, seam extraction, and pose planning
to produce complete weld trajectories from URDF or CAD inputs.

Supports two modes:
- MESH mode: Uses manifold3d + CGAL for mesh-based intersection
- OCCT mode: Uses pythonocc-core for exact CAD geometry intersection
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

# OCCT imports - optional
from .occt.occt_loader import OCCTLoader
from .occt.occt_generator import OCCTGenerator
from .occt.seam_extractor_occt import SeamExtractorOCCT


class JobPlanner:
    """Orchestrate complete weld job planning from URDF/CAD to trajectories.

    Coordinates the full pipeline:
    1. Load and process inputs (URDF or CAD files)
    2. Generate geometry (mesh or OCCT shapes)
    3. Extract seam geometry
    4. Generate torch poses

    Supports URDF/URDF, STL/STL, STEP/STEP, and mixed input combinations.
    This is the main entry point for programmatic use of the planning system.
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
        """Initialize job planner with input paths and poses.

        Parameters dict should contain:
            - work_angle_deg: Work angle in degrees
            - travel_angle_deg: Travel angle in degrees
            - gap_mm: Gap distance in millimeters
            - epsilon: Contact detection tolerance (default 0.01m for mesh, 1e-3 for OCCT)
            - num_smooth_points: Points per seam (default 100)
            - refine_iterations: Mesh subdivision iterations (default 32, mesh mode only)

        Args:
            main_path: Path to main part (URDF/STL/STEP, supports package://)
            secondary_path: Path to secondary part (URDF/STL/STEP)
            main_world_pose: Dict with 'xyz' and 'rpy' for main part pose
            secondary_world_pose: Dict with 'xyz' and 'rpy' for secondary
            parameters: Welding parameters dict
            mode: Processing mode - 'auto', 'mesh', or 'occt'
                'auto': Detect from file extensions
                'mesh': Force mesh-based processing (STL/URDF with manifold+CGAL)
                'occt': Force OCCT processing (STEP/IGES or URDF with pythonocc)

        Raises:
            ValueError: If required parameters are missing or mode invalid
            ImportError: If OCCT mode requested but pythonocc not available
        """
        self.main_path = main_path
        self.secondary_path = secondary_path
        self.main_world_transform = self._pose_to_matrix(main_world_pose)
        self.secondary_world_transform = self._pose_to_matrix(secondary_world_pose)

        self.parameters = parameters or {}

        # Determine processing mode
        if mode not in ['auto', 'mesh', 'occt']:
            raise ValueError(f"Mode must be 'auto', 'mesh', or 'occt', got '{mode}'")

        mode = 'occt'
        print(f'mode: {mode}')
        if mode == 'auto':
            self.mode = self._detect_mode(main_path, secondary_path)
        else:
            self.mode = mode

        # Set default tolerance based on mode
        if self.mode == 'occt':
            self.parameters.setdefault('epsilon', 1e-3)  # 1mm for OCCT
        else:
            self.parameters.setdefault('epsilon', 0.01)  # 10mm for mesh

        self.parameters.setdefault('num_smooth_points', 100)
        self.parameters.setdefault('refine_iterations', 32)

        required = ['work_angle_deg', 'travel_angle_deg', 'gap_mm']
        for param in required:
            if param not in self.parameters:
                raise ValueaError(f"Missing required parameter: '{param}'")

        print(f'JobPlanner initialized in {self.mode.upper()} mode')
        print(f'  work_angle={self.parameters["work_angle_deg"]}deg, '
              f'gap={self.parameters["gap_mm"]}mm, '
              f'tolerance={self.parameters["epsilon"]*1000}mm')

    def _detect_mode(self, main_path: str, secondary_path: str) -> str:
        """Auto-detect processing mode from file extensions.

        Args:
            main_path: Main part file path
            secondary_path: Secondary part file path

        Returns:
            'occt' or 'mesh'
        """
        main_ext = Path(main_path).suffix.lower()
        secondary_ext = Path(secondary_path).suffix.lower()

        occt_extensions = {'.step', '.stp', '.iges', '.igs'}
        mesh_extensions = {'.stl', '.urdf', '.xacro'}


        # Otherwise use mesh mode
        return 'mesh'

    def plan_job(self) -> list:
        """Execute complete planning pipeline.

        Returns:
            List of Seam objects with generated poses

        Raises:
            RuntimeError: If any pipeline stage fails
        """
        if self.mode == 'occt':
            return self._plan_job_occt()
        else:
            return self._plan_job_mesh()

    def _plan_job_mesh(self) -> list:
        """Execute mesh-based planning pipeline (manifold + CGAL).

        Returns:
            List of Seam objects with generated poses
        """
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
        """Execute OCCT-based planning pipeline (pythonocc).

        Returns:
            List of Seam objects with generated poses
        """
        print('Generating OCCT shapes (pythonocc-core)...')
        shape_main, shape_secondary = self._generate_occt_shapes()

        print('Extracting seams from geometry (OCCT Section)...')
        from OCC.Core.BRepBndLib import brepbndlib_Add
        from OCC.Core.Bnd import Bnd_Box

        for s, name in [(shape_main, "Main"), (shape_secondary, "Secondary")]:
            bbox = Bnd_Box()
            brepbndlib_Add(s, bbox)
            print(f"{name} BBox: X[{bbox.Get()[0]:.3f} to {bbox.Get()[3]:.3f}], Y[{bbox.Get()[1]:.3f} to {bbox.Get()[4]:.3f}], Z[{bbox.Get()[2]:.3f} to {bbox.Get()[5]:.3f}]")
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
        """Generate trimesh shells for both parts (mesh mode).

        Returns:
            Tuple of (main_mesh, secondary_mesh) as trimesh objects

        Raises:
            RuntimeError: If shell generation fails
        """
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
        """Generate OCCT shapes for both parts (OCCT mode).

        Returns:
            Tuple of (shape_main, shape_secondary) as TopoDS_Shape objects

        Raises:
            RuntimeError: If shape generation fails
        """
        shape_main = self._load_input_occt(self.main_path, self.main_world_transform)
        shape_secondary = self._load_input_occt(
            self.secondary_path, self.secondary_world_transform
        )

        print('  Main shape: OCCT TopoDS_Shape')
        print('  Secondary shape: OCCT TopoDS_Shape')

        return shape_main, shape_secondary

    def _load_input_mesh(self, path: str, world_transform: np.ndarray) -> trimesh.Trimesh:
        """Load input as trimesh (mesh mode).

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

    def _load_input_occt(self, path: str, world_transform: np.ndarray):
        """Load input as OCCT shape (OCCT mode).

        STEP/IGES files are loaded via OCCTLoader. URDF files are processed
        via URDFProcessor and OCCTGenerator.

        Args:
            path: Path to URDF, STEP, or IGES file
            world_transform: 4x4 world pose matrix to apply

        Returns:
            TopoDS_Shape ready for seam extraction

        Raises:
            RuntimeError: If loading or conversion fails
        """
        path_suffix = Path(path).suffix.lower()

        if path_suffix in ['.step', '.stp', '.iges', '.igs']:
            loader = OCCTLoader(
                cad_path=path,
                world_transform=world_transform,
            )
            return loader.shape
        else:
            # URDF/xacro file
            urdf = URDFProcessor(path)
            urdf.world_transform = world_transform
            occt_gen = OCCTGenerator(urdf.robot, world_transform)
            return occt_gen.create_shape_for_all_links()

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
