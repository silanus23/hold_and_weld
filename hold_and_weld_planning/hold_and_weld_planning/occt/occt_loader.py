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

"""OCCTLoader - Load CAD files (STEP/IGES) and convert to OCCT shapes.

Handles package:// URI resolution, applies world transforms, and provides
OCCT TopoDS_Shape objects for exact geometric seam extraction.
"""

import logging
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.IGESControl import IGESControl_Reader
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopoDS import TopoDS_Shape

from ..utils.path_utils import resolve_package_path
from ..utils.transforms import numpy_to_gp_trsf

logger = logging.getLogger(__name__)


class OCCTLoader:
    """Load CAD files and convert to OCCT shapes for weld planning pipeline.

    Handles package:// URI resolution and world pose transformation.
    Supports STEP (.step, .stp) and IGES (.iges, .igs) file formats.
    """

    def __init__(
        self,
        cad_path: str | Path,
        world_transform: NDArray | None = None,
    ) -> None:
        """Initialize OCCT loader and build shape.

        Args:
            cad_path: Path to CAD file (supports package:// URIs)
            world_transform: Global pose matrix (4x4) to apply after loading.
                Defaults to identity.

        Raises:
            ValueError: If file format unsupported or loading fails
            FileNotFoundError: If file doesn't exist
        """
        # Built here rather than in the signature: a default argument is one array shared by every
        # caller, and a caller that transforms it in place moves every later part that took the
        # default with it.
        if world_transform is None:
            world_transform = np.eye(4)

        if world_transform.shape != (4, 4):
            raise ValueError(
                f'world_transform must be 4x4, got {world_transform.shape}'
            )

        self.world_transform = world_transform

        logger.info(f'Loading CAD file: {cad_path}')
        resolved_path = resolve_package_path(cad_path)
        logger.debug(f'Resolved path: {resolved_path}')

        try:
            shape = self._load_cad_file(resolved_path)
        except Exception as e:
            logger.error(f'Failed to load CAD file: {e}')
            raise ValueError(f'Failed to load CAD file: {e}')

        self.shape = self._apply_transform(shape, world_transform)
        logger.info('CAD file loaded and transformed successfully')

    def _load_cad_file(self, file_path: Path) -> TopoDS_Shape:
        """Load CAD file based on extension (.step/.stp or .iges/.igs)."""
        suffix = file_path.suffix.lower()

        if suffix in ['.step', '.stp']:
            return self._load_step(file_path)
        elif suffix in ['.iges', '.igs']:
            return self._load_iges(file_path)
        else:
            raise ValueError(
                f'Unsupported file format: {suffix}. '
                'Supported formats: .step, .stp, .iges, .igs'
            )

    def _load_step(self, file_path: Path) -> TopoDS_Shape:
        """Load STEP file using STEPControl_Reader."""
        reader = STEPControl_Reader()
        status = reader.ReadFile(str(file_path))

        if status != IFSelect_RetDone:
            raise RuntimeError(f'Failed to read STEP file: {file_path}')

        reader.TransferRoots()
        shape = reader.OneShape()

        if shape.IsNull():
            raise RuntimeError(f'STEP file contains no valid shapes: {file_path}')

        logger.debug(f'STEP file loaded successfully: {file_path.name}')
        return shape

    def _load_iges(self, file_path: Path) -> TopoDS_Shape:
        """Load IGES file using IGESControl_Reader."""
        reader = IGESControl_Reader()
        status = reader.ReadFile(str(file_path))

        if status != IFSelect_RetDone:
            raise RuntimeError(f'Failed to read IGES file: {file_path}')

        reader.TransferRoots()
        shape = reader.OneShape()

        if shape.IsNull():
            raise RuntimeError(f'IGES file contains no valid shapes: {file_path}')

        logger.debug(f'IGES file loaded successfully: {file_path.name}')
        return shape

    def _apply_transform(
        self, shape: TopoDS_Shape, transform: NDArray
    ) -> TopoDS_Shape:
        """Apply 4x4 homogeneous transformation matrix to OCCT shape."""
        trsf = numpy_to_gp_trsf(transform)
        transformed_shape = BRepBuilderAPI_Transform(shape, trsf).Shape()

        if transformed_shape.IsNull():
            logger.error('Transform operation produced null shape')
            raise RuntimeError('Failed to apply transformation to shape')

        return transformed_shape
