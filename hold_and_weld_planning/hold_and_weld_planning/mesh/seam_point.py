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

"""SeamPoint - one ordered point on a weld seam, with its surface normals.

The unit of exchange between an extractor and PathCreator.
"""

from dataclasses import dataclass

from numpy.typing import NDArray


@dataclass
class SeamPoint:
    """Single ordered point on a weld seam with surface normal information.

    Attributes:
        position:          3D position on the geometric edge (3,).
        normal_main:       Surface normal of the base (non-edge) side (3,).
        normal_secondary:  Wall normal of the edge side (3,).
        on_edge_1:         True if mesh_1 shows a geometric edge here.
        on_edge_2:         True if mesh_2 shows a geometric edge here.
        refined_side:      Mesh carrying the edge at this point: 1 or 2.
    """

    position: NDArray
    normal_main: NDArray
    normal_secondary: NDArray
    on_edge_1: bool
    on_edge_2: bool
    refined_side: int
