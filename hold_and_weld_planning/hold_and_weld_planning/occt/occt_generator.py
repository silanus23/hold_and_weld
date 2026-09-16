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

"""OCCTGenerator - Convert URDF collision geometry to OCCT shapes.

Uses pythonocc-core to create exact geometric representations from URDF
primitives (Box, Cylinder, Sphere). Provides deterministic geometry for
precise seam extraction without mesh approximation.
"""

import logging
from typing import Any

import numpy as np
from numpy.typing import NDArray
from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
)
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Core.TopoDS import TopoDS_Shape
from urdf_parser_py.urdf import Box, Cylinder, Mesh, Sphere

from ..utils.transforms import link_poses, numpy_to_gp_trsf, origin_to_matrix

logger = logging.getLogger(__name__)


class OCCTGenerator:
    """Generate OCCT shapes from URDF collision geometry.

    Converts URDF primitives to exact OCCT geometric representations,
    applies transformations, and fuses into single compound shape.
    """

    def __init__(
        self,
        robot_object: Any,
        world_transform: NDArray | None = None,
    ) -> None:
        """Initialize OCCT generator.

        Args:
            robot_object: The self.robot object from URDFProcessor
            world_transform: Global starting pose matrix (4x4). Defaults to
                identity.

        Raises:
            ValueError: If world_transform is not 4x4, or the URDF's joint
                tree does not place every link.
        """
        # Built here rather than in the signature: a default argument is one
        # array shared by every caller, and a caller that transforms it in
        # place moves every later part that took the default with it.
        if world_transform is None:
            world_transform = np.eye(4)

        if world_transform.shape != (4, 4):
            raise ValueError(
                f'world_transform must be 4x4, got {world_transform.shape}'
            )

        self.robot = robot_object
        self.world_transform = world_transform
        # A collision origin is stated relative to its LINK, not to the model
        # root, so a multi-link part needs the joint tree walked before any of
        # its geometry can be placed.
        self.link_poses = link_poses(robot_object)

    def create_shape_for_all_links(self) -> TopoDS_Compound:
        """Combine all link geometries into single OCCT compound.

        Iterates through all links in the URDF, converts collision geometry
        to OCCT primitives, applies transforms, and fuses them together.

        Returns:
            Combined TopoDS_Compound representing entire robot

        Raises:
            RuntimeError: If any link's geometry cannot be built. A compound
                missing a link is not a smaller workpiece, it is the wrong
                one: seam extraction would go on to weld the hole the missing
                link left, so this cannot be downgraded to a skip.
        """
        compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(compound)

        total_links = len(self.robot.links)
        processed_links = 0

        logger.info(f'Creating OCCT shapes for {total_links} link(s)')

        for link in self.robot.links:
            collisions = (
                link.collisions
                if link.collisions
                else ([link.collision] if link.collision else [])
            )

            if not collisions:
                logger.debug(f'Link "{link.name}" has no collision geometry, skipping')
                continue

            try:
                link_shape = self.create_link_shape(link)
            except Exception as e:
                logger.error(f'Failed to create shape for link "{link.name}": {e}')
                raise RuntimeError(
                    f'Failed to create shape for link "{link.name}": {e}'
                )

            builder.Add(compound, link_shape)
            processed_links += 1

        logger.info(f'Successfully created shapes for {processed_links}/{total_links} link(s)')
        return compound

    def create_link_shape(self, link: Any) -> TopoDS_Shape:
        """Create OCCT shape for all collision elements in a link.

        Args:
            link: URDF link object

        Returns:
            TopoDS_Shape representing union of all collision geometries

        Raises:
            ValueError: If geometry type is unsupported or has invalid dimensions
        """
        collisions = (
            link.collisions
            if link.collisions
            else ([link.collision] if link.collision else [])
        )

        shapes = []

        logger.debug(f'Processing {len(collisions)} collision element(s) for link "{link.name}"')

        for idx, collision in enumerate(collisions):
            geom = collision.geometry

            if isinstance(geom, Box):
                if len(geom.size) != 3:
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Box size must be [x, y, z], got {geom.size}'
                    )

                dx, dy, dz = geom.size
                pnt = gp_Pnt(-dx/2, -dy/2, -dz/2)
                shape = BRepPrimAPI_MakeBox(pnt, dx, dy, dz).Shape()

            elif isinstance(geom, Cylinder):
                if geom.radius <= 0 or geom.length <= 0:
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Cylinder dimensions must be positive: '
                        f'radius={geom.radius}, length={geom.length}'
                    )

                ax = gp_Ax2(gp_Pnt(0, 0, -geom.length/2), gp_Dir(0, 0, 1))
                shape = BRepPrimAPI_MakeCylinder(ax, geom.radius, geom.length).Shape()

            elif isinstance(geom, Sphere):
                if geom.radius <= 0:
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Sphere radius must be positive: {geom.radius}'
                    )

                shape = BRepPrimAPI_MakeSphere(geom.radius).Shape()

            elif isinstance(geom, Mesh):
                raise ValueError(
                    f"Link '{link.name}' collision {idx}: "
                    f'Mesh geometry is not supported'
                )

            else:
                raise ValueError(
                    f"Link '{link.name}' collision {idx}: "
                    f'Unsupported geometry type: {type(geom).__name__}'
                )

            link_T = self.link_poses.get(link.name, np.eye(4))
            local_T = origin_to_matrix(collision.origin)
            absolute_T = self.world_transform @ link_T @ local_T

            trsf = numpy_to_gp_trsf(absolute_T)
            transformed_shape = BRepBuilderAPI_Transform(shape, trsf).Shape()
            shapes.append(transformed_shape)

        if len(shapes) == 0:
            logger.warning(f'Link "{link.name}" produced no valid shapes')
            raise ValueError(f'Link "{link.name}" has no valid collision geometry')

        if len(shapes) == 1:
            return shapes[0]

        logger.debug(f'Fusing {len(shapes)} shape(s) for link "{link.name}"')
        result = shapes[0]
        for i, shape in enumerate(shapes[1:], start=1):
            fused = BRepAlgoAPI_Fuse(result, shape)
            result = fused.Shape()

            if result.IsNull():
                logger.error(
                    f'Fuse operation failed for link "{link.name}" at shape {i+1}/{len(shapes)}'
                )
                raise ValueError(f'Failed to fuse shapes for link "{link.name}"')

        return result
