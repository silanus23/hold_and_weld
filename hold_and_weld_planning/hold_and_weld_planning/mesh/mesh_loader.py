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

"""MeshLoader - Load and process STL meshes for weld planning.

Handles package:// URI resolution, mesh refinement, and manifold conversion.
"""

import logging
from pathlib import Path

import manifold3d
import numpy as np
from numpy.typing import NDArray
import trimesh

from .params import MeshLoadParams
from ..utils.path_utils import resolve_package_path

logger = logging.getLogger(__name__)


class MeshLoader:
    """Load STL mesh files and convert to manifold for weld planning pipeline.

    Attributes:
        manifold: Processed manifold3d object ready for boolean operations
        world_transform: Applied 4x4 transformation matrix
        refine_iterations: Number of subdivision iterations used
    """

    def __init__(
        self,
        mesh_path: str | Path,
        world_transform: NDArray | None = None,
        refine_iterations: int = MeshLoadParams.refine_iterations,
    ) -> None:
        """Initialize mesh loader and build manifold.

        Args:
            mesh_path: Path to mesh file (supports package:// URIs)
            world_transform: Global pose matrix (4x4) to apply after loading.
                Defaults to identity.
            refine_iterations: Pieces each edge is split into; see MeshLoadParams

        Raises:
            ValueError: If mesh loading or conversion fails
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

        refine_iterations = MeshLoadParams.from_dict(
            {'refine_iterations': refine_iterations}).refine_iterations

        self.world_transform = world_transform
        self.refine_iterations = refine_iterations

        logger.debug(f'Loading mesh from {mesh_path}')
        resolved_path = resolve_package_path(mesh_path)
        logger.debug(f'Resolved path: {resolved_path}')

        try:
            mesh = trimesh.load(resolved_path)
        except Exception as e:
            raise ValueError(f'Failed to load mesh from {resolved_path}: {e}') from e

        # A file holding several solids loads as a Scene. Each is kept as its own mesh and they
        # are unioned below: concatenating them into one mesh would leave any overlap
        # self-intersecting.
        if isinstance(mesh, trimesh.Scene):
            meshes = [m for m in mesh.dump() if isinstance(m, trimesh.Trimesh)]
        elif isinstance(mesh, trimesh.Trimesh):
            meshes = [mesh]
        else:
            raise ValueError(
                f'{resolved_path} holds no triangle mesh (loaded as {type(mesh).__name__})')

        if not meshes or sum(len(m.vertices) for m in meshes) == 0:
            raise ValueError(f'Loaded mesh has no vertices: {resolved_path}')

        logger.info(
            f'Loaded mesh: {len(meshes)} solid(s), '
            f'{sum(len(m.vertices) for m in meshes)} vertices, '
            f'{sum(len(m.faces) for m in meshes)} faces'
        )
        self.manifold = self._build_manifold(meshes)
        if refine_iterations > 0:
            logger.info(
                f'Mesh loaded and converted to manifold (refined {refine_iterations} iterations)'
            )
        else:
            logger.info('Mesh loaded and converted to manifold (no refinement)')

    def _build_manifold(self, meshes: list[trimesh.Trimesh]) -> manifold3d.Manifold:
        """Union the meshes as manifolds, refine for density, and apply world transform."""
        manifold_obj = manifold3d.Manifold()
        for index, mesh in enumerate(meshes):
            try:
                solid = manifold3d.Manifold(
                    manifold3d.Mesh(
                        vert_properties=np.array(mesh.vertices, dtype=np.float64),
                        tri_verts=np.array(mesh.faces, dtype=np.int32),
                    )
                )
            except Exception as e:
                raise ValueError(f'Failed to convert mesh {index} to manifold: {e}') from e

            # manifold3d does not raise on a malformed mesh - it silently returns an empty
            # Manifold whose error status would otherwise only surface much later, inside
            # whatever boolean op first touches it.
            status = solid.status()
            if status != manifold3d.Error.NoError:
                raise ValueError(f'Mesh {index} failed manifold conversion: {status}')

            manifold_obj += solid

        # Subdivide to increase vertex density for smoother seam extraction
        if self.refine_iterations > 0:
            logger.debug(f'Refining mesh with {self.refine_iterations} iterations')
            manifold_obj = manifold_obj.refine(self.refine_iterations)

        # Apply world transform (manifold3d uses 3x4 matrix: [R|t])
        mat_3x4 = self.world_transform[:3, :].tolist()
        manifold_obj = manifold_obj.transform(mat_3x4)

        return manifold_obj
