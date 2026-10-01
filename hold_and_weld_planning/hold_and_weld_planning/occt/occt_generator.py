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
precise seam extraction without mesh approximation. Every link of a part is
fused into one solid, so a face where two links meet is interior rather than
a seam.
"""

import logging
from typing import Any

from numpy.typing import NDArray
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import (
    BRepPrimAPI_MakeBox,
    BRepPrimAPI_MakeCylinder,
    BRepPrimAPI_MakeSphere,
)
from OCC.Core.gp import gp_Ax2, gp_Dir, gp_Pnt
from OCC.Core.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCC.Core.TopoDS import TopoDS_Shape
from urdf_parser_py.urdf import Box, Cylinder, Mesh, Sphere

from ..utils.transforms import (
    as_world_transform, link_poses, numpy_to_gp_trsf, origin_to_matrix)

logger = logging.getLogger(__name__)


class OCCTGenerator:
    """Generate OCCT shapes from URDF collision geometry.

    Converts URDF primitives to exact OCCT geometric representations,
    applies transformations, and fuses them into a single solid.
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
            ValueError: If world_transform is not a finite 4x4, or the URDF's joint
                tree does not place every link.
        """
        world_transform = as_world_transform(world_transform)

        self.robot = robot_object
        self.world_transform = world_transform
        # A collision origin is stated relative to its LINK, not to the model root, so a multi-link
        # part needs the joint tree walked before any of its geometry can be placed.
        self.link_poses = link_poses(robot_object)

    def create_shape_for_all_links(self) -> TopoDS_Shape:
        """Fuse all link geometries into a single OCCT shape.

        Iterates through all links in the URDF, converts collision geometry
        to OCCT primitives, applies transforms, and fuses them together.

        Returns:
            Fused TopoDS_Shape representing the entire part

        Raises:
            RuntimeError: If any link's geometry cannot be built, or the links
                cannot be fused. A shape missing a link is not a smaller
                workpiece, it is the wrong one: seam extraction would go on to
                weld the hole the missing link left, so this cannot be
                downgraded to a skip.
            ValueError: If no link has collision geometry, or a collision
                element is unsupported or malformed.
        """
        link_shapes = []

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
            except (ValueError, RuntimeError):
                # Already name the link; ValueError stays one so a malformed spec reads as config.
                raise
            except Exception as e:
                raise RuntimeError(
                    f'Failed to create shape for link "{link.name}": {e}'
                ) from e

            link_shapes.append(link_shape)
            processed_links += 1

        if not link_shapes:
            raise ValueError(
                'URDF has no collision geometry on any link; seams are extracted from '
                '<collision> elements, not <visual>'
            )

        logger.info(f'Successfully created shapes for {processed_links}/{total_links} link(s)')
        return self._fuse(link_shapes, 'links')

    @staticmethod
    def _fuse(shapes: list[TopoDS_Shape], what: str) -> TopoDS_Shape:
        """Fuse shapes into one and merge the faces the fuse left split.

        A plain compound keeps each link's own faces, so two links meeting flush leave a face
        pair where they touch and an edge across each face they share: both read to the
        extractor as a joint. The fuse removes the touching faces; unifying then merges the
        coplanar faces either side of the old boundary so no edge is left along it.

        Raises:
            RuntimeError: If the fuse fails.
        """
        if len(shapes) == 1:
            return shapes[0]

        result = shapes[0]
        for shape in shapes[1:]:
            fuse = BRepAlgoAPI_Fuse(result, shape)
            if not fuse.IsDone() or fuse.Shape().IsNull():
                raise RuntimeError(f'Failed to fuse {what}')
            result = fuse.Shape()

        unify = ShapeUpgrade_UnifySameDomain(result, True, True, False)
        unify.Build()
        return unify.Shape()

    def create_link_shape(self, link: Any) -> TopoDS_Shape:
        """Create OCCT shape for all collision elements in a link.

        Args:
            link: URDF link object

        Returns:
            TopoDS_Shape representing union of all collision geometries

        Raises:
            ValueError: If geometry type is unsupported or has invalid dimensions
            RuntimeError: If the link's collision shapes cannot be fused
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

                if not all(s > 0 for s in geom.size):
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Box size must be positive: {geom.size}'
                    )

                dx, dy, dz = geom.size
                pnt = gp_Pnt(-dx/2, -dy/2, -dz/2)
                shape = BRepPrimAPI_MakeBox(pnt, dx, dy, dz).Shape()

            elif isinstance(geom, Cylinder):
                if not (geom.radius > 0 and geom.length > 0):
                    raise ValueError(
                        f"Link '{link.name}' collision {idx}: "
                        f'Cylinder dimensions must be positive: '
                        f'radius={geom.radius}, length={geom.length}'
                    )

                ax = gp_Ax2(gp_Pnt(0, 0, -geom.length/2), gp_Dir(0, 0, 1))
                shape = BRepPrimAPI_MakeCylinder(ax, geom.radius, geom.length).Shape()

            elif isinstance(geom, Sphere):
                if not geom.radius > 0:
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

            link_T = self.link_poses[link.name]
            local_T = origin_to_matrix(collision.origin)
            absolute_T = self.world_transform @ link_T @ local_T

            trsf = numpy_to_gp_trsf(absolute_T)
            transformed_shape = BRepBuilderAPI_Transform(shape, trsf).Shape()
            shapes.append(transformed_shape)

        if len(shapes) == 0:
            raise ValueError(f'Link "{link.name}" has no valid collision geometry')

        logger.debug(f'Fusing {len(shapes)} shape(s) for link "{link.name}"')
        return self._fuse(shapes, f'shapes for link "{link.name}"')
