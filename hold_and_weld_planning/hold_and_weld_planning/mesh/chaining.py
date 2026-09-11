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

"""Turn loose contact-boundary edges into ordered, oriented seam polylines.

Four steps, in the order `SeamExtractorMesh.extract_chains` applies them:
`loops`, `oriented`, `drop_coincident`, `stitch`.

Plain functions of their arguments, reading only vertex positions and
`SeamExtractorMeshParams` tuning - kept separate from `SeamExtractorMesh` so
they can be tested against plain arrays, no mesh or extractor required.
"""

import logging
from typing import Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import KDTree
import trimesh

from .params import SeamExtractorMeshParams

logger = logging.getLogger(__name__)

COINCIDENT_SPACING_FRACTION = 1e-6


def loops(
    side: int,
    boundary: List[Tuple[int, int]],
    cfg: SeamExtractorMeshParams,
) -> List[Tuple[List[int], bool]]:
    """Walk boundary edges into ordered chains of vertex indices.

    A boundary vertex normally joins exactly two edges, so the walk has one
    way to continue. Degree above two means two boundary curves cross, and
    nothing local says which branch continues which seam, so the walk stops
    there rather than guessing - the curves through it come out as open
    chains instead of one spliced loop.

    Args:
        side: Naming only, for logs.

    Returns:
        (vertex_indices, is_closed) per chain, shorter chains discarded. A
        closed chain does not repeat its first vertex at the end.
    """
    adjacency: Dict[int, List[int]] = {}
    for a, b in boundary:
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)

    junctions = {v for v, n in adjacency.items() if len(n) != 2}
    if junctions:
        logger.info(
            f'mesh_{side} contact boundary has {len(junctions)} junction '
            f'vertex(es) of degree != 2; cutting into open chains there'
        )

    chains: List[Tuple[List[int], bool]] = []
    used: set = set()

    def walk(start: int, first: int) -> List[int]:
        chain = [start, first]
        used.add((min(start, first), max(start, first)))
        previous, current = start, first
        while current not in junctions:
            options = [v for v in adjacency[current] if v != previous]
            if not options:
                break
            nxt = options[0]
            key = (min(current, nxt), max(current, nxt))
            if key in used:
                break
            used.add(key)
            chain.append(nxt)
            previous, current = current, nxt
        return chain

    # Junctions first: each branch leaving one is a separate open chain.
    for junction in junctions:
        for neighbour in adjacency[junction]:
            key = (min(junction, neighbour), max(junction, neighbour))
            if key not in used:
                chains.append((walk(junction, neighbour), False))

    # Whatever is left touches no junction, so it can only be a closed loop.
    for start in adjacency:
        for neighbour in adjacency[start]:
            key = (min(start, neighbour), max(start, neighbour))
            if key in used:
                continue
            chain = walk(start, neighbour)
            closed = len(chain) > 2 and chain[-1] == chain[0]
            if closed:
                chain = chain[:-1]
            chains.append((chain, closed))

    return [
        (chain, closed)
        for chain, closed in chains
        if len(chain) >= cfg.min_loop_points
    ]


def oriented(
    mesh: trimesh.Trimesh,
    loop: List[int],
    contact: NDArray,
    wall_normal: NDArray,
) -> List[int]:
    """Orient a loop so the contact region lies consistently to one side.

    Args:
        wall_normal: Unit normal of the wall rising out of the joint at
            `loop[0]`.

    Returns:
        The loop, reversed after its first vertex when it ran the wrong way.
    """
    positions = mesh.vertices[loop]
    centroid = mesh.triangles_center[contact].mean(axis=0)

    tangent = positions[1] - positions[-1]
    norm = np.linalg.norm(tangent)
    if norm < 1e-12:
        return loop
    tangent /= norm

    inward = np.cross(tangent, wall_normal)
    if np.dot(inward, centroid - positions[0]) < 0.0:
        return [loop[0]] + loop[1:][::-1]
    return loop


def drop_coincident(
    pieces: List[Tuple[NDArray, bool]], cfg: SeamExtractorMeshParams
) -> List[Tuple[NDArray, bool]]:
    """Drop a piece that lies on top of a better-sampled one.

    On an edge-to-edge joint both parts terminate on the same curve and it
    would be welded twice. Coincident pieces sit within epsilon along their
    whole length - the same condition that defined contact, so no new
    tolerance is introduced.

    Returns:
        The same list less the pieces another piece already covers.
    """
    # One tree per PIECE, not one per comparison: the inner loop below queries
    # the same piece's tree once for every other piece, and rebuilding it each
    # time is quadratic in the number of pieces for no gain.
    trees = [KDTree(positions) for positions, _ in pieces]

    keep: List[Tuple[NDArray, bool]] = []
    for index, (positions, is_closed) in enumerate(pieces):
        redundant = False
        for other_index, (others, _) in enumerate(pieces):
            if other_index == index or len(others) < len(positions):
                continue
            if len(others) == len(positions) and other_index > index:
                continue
            covered = trees[other_index].query(positions)[0].max()
            if float(covered) <= cfg.epsilon:
                redundant = True
                break
        if redundant:
            logger.info(
                f'Dropping {len(positions)}-point piece: another piece '
                'samples the same curve more finely'
            )
        else:
            keep.append((positions, is_closed))
    return keep


def stitch(
    pieces: List[Tuple[NDArray, bool]], cfg: SeamExtractorMeshParams
) -> List[Tuple[NDArray, bool]]:
    """Join polylines from different meshes end to end into one chain.

    Each mesh contributes only the portion of the seam where it terminates,
    and the two share no vertices, so the pieces meet with a gap of roughly
    one segment. Closing those gaps gives one chain whose ownership
    alternates along its length.

    Returns:
        Closed chains pass through untouched; an open chain that closes on
        itself is returned closed, with its repeated last point removed.
    """
    open_pieces = [list(p) for p, closed in pieces if not closed]
    result = [(p, True) for p, closed in pieces if closed]

    def span(piece: List[NDArray]) -> float:
        """Measure the sampling density local to the ENDS of one piece.

        At the ends, not pooled over the whole piece: a stitched chain
        crosses meshes sampled at very different densities, so a pooled
        median would be dominated by the finer mesh and judge a coarse
        end's gap far too tight.
        """
        if len(piece) < 2:
            return 0.0
        array = np.asarray(piece)
        window = min(cfg.min_loop_points, len(array))
        head = np.linalg.norm(np.diff(array[:window], axis=0), axis=1)
        tail = np.linalg.norm(np.diff(array[-window:], axis=0), axis=1)
        return float(max(np.median(head), np.median(tail)))

    # A lone open piece still needs to reach the closure test below, so no
    # early return here - the merge loop already no-ops on one piece.

    # Tolerance is per PAIR being joined, not pooled across all pieces: mesh
    # densities vary widely, so a pooled median would refuse real handoffs.
    merged = True
    while merged and len(open_pieces) > 1:
        merged = False
        for i in range(len(open_pieces)):
            for j in range(len(open_pieces)):
                if i == j:
                    continue
                a, b = open_pieces[i], open_pieces[j]
                options = (
                    (np.linalg.norm(a[-1] - b[0]), False, False),
                    (np.linalg.norm(a[-1] - b[-1]), False, True),
                    (np.linalg.norm(a[0] - b[0]), True, False),
                    (np.linalg.norm(a[0] - b[-1]), True, True),
                )
                gap, flip_a, flip_b = min(options, key=lambda o: o[0])
                tolerance = cfg.stitch_gap_factor * max(span(a), span(b))
                if gap > tolerance:
                    continue
                logger.info(
                    f'Stitched a {len(a)}-point and a {len(b)}-point piece; '
                    f'gap {gap * 1000:.2f}mm within tolerance '
                    f'{tolerance * 1000:.2f}mm'
                )
                head = a[::-1] if flip_a else a
                tail = b[::-1] if flip_b else b
                open_pieces[i] = head + tail
                open_pieces.pop(j)
                merged = True
                break
            if merged:
                break

    for piece in open_pieces:
        array = np.asarray(piece)
        spacing = span(piece)
        gap = float(np.linalg.norm(array[0] - array[-1]))
        closed = (
            len(array) >= cfg.min_loop_points
            and gap <= cfg.stitch_gap_factor * spacing
        )
        # Closing doesn't make the last point redundant - the gap that closed
        # it is a real step. Only a last point landed exactly ON the first is
        # a repeat (the case `loops` already strips by vertex index).
        if closed and gap <= COINCIDENT_SPACING_FRACTION * spacing:
            array = array[:-1]
        result.append((array, closed))
    return result
