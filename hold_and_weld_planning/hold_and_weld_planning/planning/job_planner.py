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

from dataclasses import fields
import logging
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
from OCC.Core.TopoDS import TopoDS_Shape
import trimesh

from .weld_planner import WeldPlanner, WeldPlannerParams
from ..core.seam import Seam
from ..mesh.mesh_loader import MeshLoader
from ..mesh.params import MeshLoadParams, PathCreatorParams, SeamExtractorMeshParams
from ..mesh.seam_extractor_mesh import SeamExtractorMesh
from ..mesh.shell_generator import ShellGenerator
from ..occt.occt_generator import OCCTGenerator
from ..occt.occt_loader import OCCTLoader
from ..occt.seam_extractor_occt import SeamExtractorOCCT, SeamExtractorOCCTParams
from ..urdf.urdf_processor import URDFProcessor
from ..utils.transforms import xyz_rpy_to_matrix

logger = logging.getLogger(__name__)

OCCT_EXTENSIONS = {'.step', '.stp', '.iges', '.igs'}
MESH_EXTENSIONS = {'.stl'}
URDF_EXTENSIONS = {'.urdf', '.xacro'}

# Every key some stage of either pipeline reads from the shared parameter dict. Each stage ignores
# the others' keys by design, so a misspelled key is ignored by all of them, silently, unless
# something checks it against the whole set.
KNOWN_PARAMETERS = frozenset(
    spec.name
    for params in (MeshLoadParams, SeamExtractorMeshParams, PathCreatorParams,
                   SeamExtractorOCCTParams, WeldPlannerParams)
    for spec in fields(params)
)

# Keys that once took millimetres. Their values would now be read as metres, 1000x too large, so
# an old config is refused by name rather than warned about as merely unknown.
RENAMED_PARAMETERS = {
    'gap_mm': 'gap',
    'waypoint_spacing_mm': 'waypoint_spacing',
    'path_tolerance_mm': 'path_tolerance',
}


class JobPlanner:
    """Orchestrate complete weld job planning from URDF/CAD to trajectories.

    Main entry point for programmatic use of the planning system.
    Supports URDF, STL, and STEP inputs with automatic mode detection.
    """

    def __init__(
        self,
        main_path: str,
        secondary_path: str,
        main_world_pose: dict[str, list] | None = None,
        secondary_world_pose: dict[str, list] | None = None,
        parameters: dict[str, Any] | None = None,
        mode: str = 'auto',
    ) -> None:
        """Initialize job planner."""
        self.main_path = main_path
        self.secondary_path = secondary_path
        self.main_world_transform = self._pose_to_matrix(main_world_pose)
        self.secondary_world_transform = self._pose_to_matrix(secondary_world_pose)

        self.parameters = dict(parameters or {})

        renamed = sorted(set(self.parameters) & set(RENAMED_PARAMETERS))
        if renamed:
            raise ValueError(
                'Parameter(s) renamed and now in metres: '
                + ', '.join(f'{old} -> {RENAMED_PARAMETERS[old]}' for old in renamed))

        unknown = sorted(set(self.parameters) - KNOWN_PARAMETERS)
        if unknown:
            logger.warning(
                f'Unknown parameter key(s) {unknown} are read by no stage and have no effect; '
                'check the spelling against PARAMS.md'
            )

        if mode not in ['auto', 'mesh', 'occt']:
            raise ValueError(f"Mode must be 'auto', 'mesh', or 'occt', got '{mode}'")

        if mode == 'auto':
            self.mode = self._detect_mode(main_path, secondary_path)
            logger.info(f'Auto-detected mode: {self.mode.upper()}')
        else:
            self.mode = mode

        self._validate_inputs(main_path, secondary_path)

        if self.mode == 'occt':
            self.parameters.setdefault('epsilon', SeamExtractorOCCTParams.epsilon)
        else:
            self.parameters.setdefault('epsilon', SeamExtractorMeshParams.epsilon)

        # Every stage's parameters are validated here, before any geometry is loaded, so a bad
        # value fails at once rather than after the shells are built and the seams extracted. The
        # extractors rebuild theirs from the same dict.
        self.weld_planner = WeldPlanner(self.parameters)
        if self.mode == 'occt':
            SeamExtractorOCCTParams.from_dict(self.parameters)
        else:
            self.refine_iterations = MeshLoadParams.from_dict(self.parameters).refine_iterations
            SeamExtractorMeshParams.from_dict(self.parameters)
            PathCreatorParams.from_dict(self.parameters)

            if 'refine_iterations' in self.parameters and self.refine_iterations == 0:
                logger.warning(
                    'refine_iterations=0: seam points are mesh vertices, so a part whose edges '
                    'are long compared with the joint cannot represent where the seam starts '
                    'and ends on them, and that portion is silently dropped. Use 16-32 unless '
                    'the mesh is already fine at the joint.'
                )

        logger.info(f'JobPlanner initialized in {self.mode.upper()} mode')
        logger.info(f'Parameters: work_angle={self.parameters["work_angle_deg"]}deg, '
                    f'travel_angle={self.parameters["travel_angle_deg"]}deg, '
                    f'gap={self.parameters["gap"]*1000:.2f}mm, '
                    f'tolerance={self.parameters["epsilon"]*1000:.3f}mm')

    def _detect_mode(self, main_path: str, secondary_path: str) -> str:
        """Auto-detect processing mode (mesh vs occt) from file extensions."""
        main_ext = Path(main_path).suffix.lower()
        secondary_ext = Path(secondary_path).suffix.lower()

        if main_ext in OCCT_EXTENSIONS or secondary_ext in OCCT_EXTENSIONS:
            return 'occt'

        if (main_ext in MESH_EXTENSIONS | URDF_EXTENSIONS
                and secondary_ext in MESH_EXTENSIONS | URDF_EXTENSIONS):
            return 'mesh'

        raise ValueError(
            f'Cannot auto-detect mode: unsupported file extension(s)'
            f" '{main_ext}'/'{secondary_ext}'. "
            f'Supported: {sorted(OCCT_EXTENSIONS)} for OCCT mode, '
            f'{sorted(MESH_EXTENSIONS | URDF_EXTENSIONS)} for mesh mode. '
            f"Set mode explicitly via the 'mode' parameter."
        )

    def _validate_inputs(self, main_path: str, secondary_path: str) -> None:
        """Check both inputs are readable by the resolved pipeline.

        Detection settles on a mode from whichever input is the more specific,
        so a STEP paired with an STL selects OCCT and the STL then reaches the
        URDF branch of the loader, where xacro fails on it as malformed XML.
        Name the offending file here instead.

        Raises:
            ValueError: If either input's extension is foreign to `self.mode`.
        """
        allowed = URDF_EXTENSIONS | (
            OCCT_EXTENSIONS if self.mode == 'occt' else MESH_EXTENSIONS
        )

        for label, path in (('main', main_path), ('secondary', secondary_path)):
            extension = Path(path).suffix.lower()
            if extension not in allowed:
                raise ValueError(
                    f'{self.mode.upper()} mode cannot read the {label} part '
                    f"'{path}': extension '{extension}' is not one of "
                    f'{sorted(allowed)}. Both parts must come from the same '
                    f'family, or set the mode explicitly.'
                )

    def plan_job(self) -> list[Seam]:
        """Execute complete planning pipeline.

        Returns:
            The seams, each with its poses filled in. Empty when no seam is found.

        Raises:
            RuntimeError: If a pipeline stage fails, including any single seam
                failing to extract or plan
            ValueError: If an input or parameter is invalid
            FileNotFoundError: If an input file does not exist
        """
        if self.mode == 'occt':
            return self._plan_job_occt()
        else:
            return self._plan_job_mesh()

    def _plan_job_mesh(self) -> list[Seam]:
        """Execute mesh-based planning pipeline using manifold3d and trimesh."""
        logger.info('Generating mesh shells (manifold3d)...')
        mesh_main, mesh_secondary = self._generate_shells()

        logger.debug(f'Mesh 1: watertight={mesh_main.is_watertight}, '
                     f'faces={len(mesh_main.faces)}, bounds={mesh_main.bounds}')
        logger.debug(f'Mesh 2: watertight={mesh_secondary.is_watertight}, '
                     f'faces={len(mesh_secondary.faces)}, bounds={mesh_secondary.bounds}')

        logger.info('Extracting seams as the contact boundary...')
        extractor = SeamExtractorMesh(
            mesh_main, mesh_secondary, self.parameters
        )
        seams = extractor.extract_seams()

        if not seams:
            logger.warning('No seams detected')
            return []

        logger.info(f'Detected {len(seams)} seam(s)')
        return self._generate_poses(seams)

    def _plan_job_occt(self) -> list[Seam]:
        """Execute OCCT-based planning pipeline using pythonocc-core."""
        logger.info('Generating OCCT shapes (pythonocc-core)...')
        shape_main, shape_secondary = self._generate_occt_shapes()

        logger.info('Extracting seams from geometry (OCCT)...')
        seam_extractor = SeamExtractorOCCT(shape_main, shape_secondary, self.parameters)
        seams = seam_extractor.extract_seams()

        if not seams:
            logger.warning('No seams detected')
            return []

        logger.info(f'Detected {len(seams)} seam(s)')
        return self._generate_poses(seams)

    def _generate_poses(self, seams: list[Seam]) -> list[Seam]:
        """Run WeldPlanner over every extracted seam.

        Raises:
            RuntimeError: If any seam fails to plan. Every seam is attempted first so the error
                names them all; a job missing a seam is the wrong job, not a smaller one.
        """
        logger.info('Generating weld poses...')

        failures = []
        for idx, seam in enumerate(seams):
            try:
                self.weld_planner.generate_seam(seam)
                num_poses = len(seam.poses) if seam.poses else 0
                logger.debug(f'Seam {idx}: {num_poses} pose(s) generated')
            except Exception as e:
                failures.append(f'seam {idx}: {e}')
                logger.error(f'Failed to generate poses for {failures[-1]}')

        if failures:
            raise RuntimeError(
                f'{len(failures)} of {len(seams)} seam(s) could not be planned: '
                + '; '.join(failures)
            )

        logger.info(f'Planned {len(seams)} seam(s)')
        return seams

    def _generate_shells(self) -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
        """Generate watertight trimesh shells for both parts."""
        mesh_main = self._load_input_mesh(self.main_path, self.main_world_transform)
        mesh_secondary = self._load_input_mesh(
            self.secondary_path, self.secondary_world_transform
        )

        logger.info(f'Main mesh: {len(mesh_main.vertices)} vertices, {len(mesh_main.faces)} faces')
        logger.info(
            f'Secondary mesh: {len(mesh_secondary.vertices)} vertices, '
            f'{len(mesh_secondary.faces)} faces'
        )

        return mesh_main, mesh_secondary

    def _generate_occt_shapes(self) -> tuple[TopoDS_Shape, TopoDS_Shape]:
        """Generate OCCT TopoDS_Shape objects for both parts."""
        shape_main = self._load_input_occt(self.main_path, self.main_world_transform)
        shape_secondary = self._load_input_occt(
            self.secondary_path, self.secondary_world_transform
        )
        return shape_main, shape_secondary

    def _load_input_mesh(self, path: str, world_transform: NDArray) -> trimesh.Trimesh:
        """Load URDF or STL as trimesh and apply world transform."""
        refine_iterations = self.refine_iterations

        if Path(path).suffix.lower() in MESH_EXTENSIONS:
            loader = MeshLoader(
                mesh_path=path,
                world_transform=world_transform,
                refine_iterations=refine_iterations,
            )
            manifold_obj = loader.manifold
        else:
            urdf = URDFProcessor(path)
            shell_gen = ShellGenerator(
                urdf.robot, world_transform, refine_iterations=refine_iterations
            )
            manifold_obj = shell_gen.create_shells_for_all_links()

        mesh_data = manifold_obj.to_mesh()

        return trimesh.Trimesh(
            vertices=mesh_data.vert_properties, faces=mesh_data.tri_verts
        )

    def _load_input_occt(self, path: str, world_transform: NDArray) -> TopoDS_Shape:
        """Load URDF, STEP, or IGES as OCCT shape and apply world transform."""
        path_suffix = Path(path).suffix.lower()

        if path_suffix in OCCT_EXTENSIONS:
            loader = OCCTLoader(
                cad_path=path,
                world_transform=world_transform,
            )
            return loader.shape
        else:
            urdf = URDFProcessor(path)
            occt_gen = OCCTGenerator(urdf.robot, world_transform)
            return occt_gen.create_shape_for_all_links()

    @staticmethod
    def _pose_to_matrix(pose: dict[str, list] | None) -> NDArray:
        """Convert an xyz/rpy dict to a 4x4 homogeneous transform; None is the identity.

        Raises:
            ValueError: If pose is not a dict, or its xyz or rpy is not 3 finite numbers.
        """
        if pose is None:
            return np.eye(4)
        if not isinstance(pose, dict):
            raise ValueError(f'world_pose must be a mapping with xyz and rpy, got {pose!r}')
        return xyz_rpy_to_matrix(pose.get('xyz', [0.0, 0.0, 0.0]),
                                 pose.get('rpy', [0.0, 0.0, 0.0]))
