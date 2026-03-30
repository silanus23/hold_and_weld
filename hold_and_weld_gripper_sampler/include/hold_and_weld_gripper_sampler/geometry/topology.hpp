// Copyright 2026 Berkan Tali
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Current design uses std::vector for topology storage with O(1) element access.
// IDs are sequential (0, 1, 2, ...) and correspond directly to vector indices.
//
// Future optimizations for complex parts (100+ surfaces):
// - Add R-tree spatial index for proximity queries
// - Use OCCT's BVH for collision acceleration
// - Add octree for spatial partitioning

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__TOPOLOGY_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__TOPOLOGY_HPP_

#include <utility>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

/**
 * @brief Classification of edge geometry type
 */
enum class EdgeType
{
  LINE,     // Straight line segment
  CIRCLE,   // Circular arc
  SPLINE    // B-spline, Bezier, ellipse, or other curved edge
};

/**
 * @brief Represents a corner (vertex) in the object topology.
 *
 * Stores position and connectivity to adjacent edges and surfaces.
 * All data is in world frame after STEP file transformation.
 */
struct Corner
{
  /**
   * @brief 3D position of the corner [world frame]
   */
  gp_Pnt position;

  /**
   * @brief IDs of edges connected to this corner (0-indexed)
   */
  std::vector<int> connected_edges;

  /**
   * @brief IDs of surfaces that share this corner (0-indexed)
   */
  std::vector<int> connected_surfaces;

  Corner() = default;
};

/**
 * @brief Represents an edge in the object topology.
 *
 * Stores the two corner endpoints and connectivity to adjacent surfaces.
 */
struct Edge
{
  /**
   * @brief OCCT edge handle
   *
   * Contains the underlying CAD edge geometry.
   * Use BRep_Tool to extract geometric properties.
   */
  TopoDS_Edge edge;

  /**
   * @brief IDs of the two corners that define this edge (0-indexed)
   *
   * Pair format: (corner_id_1, corner_id_2)
   * Order is arbitrary - edge is undirected.
   */
  std::pair<int, int> corner_ids;

  /**
   * @brief IDs of surfaces that share this edge (0-indexed)
   *
   * Typically 1-2 surfaces (boundary edge has 1, interior edge has 2).
   * Non-manifold geometry may have more.
   */
  std::vector<int> connected_surfaces;

  /**
   * @brief Geometric type of this edge
   *
   * Classified as LINE, CIRCLE, or SPLINE based on underlying curve geometry.
   */
  EdgeType type;

  Edge()
  : type(EdgeType::LINE) {}
};

/**
 * @brief Represents a surface (face) in the object topology.
 *
 * Stores OCCT face handle, geometric properties, and connectivity.
 * All geometric data is in world frame after transformation.
 */
struct Surface
{
  /**
   * @brief OCCT face handle
   *
   * Contains the underlying CAD surface geometry.
   * Use BRep_Tool to extract geometric properties.
   */
  TopoDS_Face face;

  /**
   * @brief Surface normal vector [world frame, unit length]
   *
   * CRITICAL: Must be orientation-corrected!
   * If face.Orientation() == TopAbs_REVERSED, multiply by -1.
   */
  gp_Vec normal;

  /**
   * @brief Geometric center of the surface [world frame]
   *
   * Used for spatial queries and visualization.
   */
  gp_Pnt center;

  /**
   * @brief IDs of edges bounding this surface (0-indexed)
   */
  std::vector<int> edge_ids;

  /**
   * @brief IDs of corners belonging to this surface (0-indexed)
   */
  std::vector<int> corner_ids;

  /**
   * @brief Flag indicating if surface has inner holes (inner wires)
   *
   * True if the surface has more than one wire (outer boundary + inner holes).
   * Example: A washer has one outer circle and one inner hole.
   */
  bool has_inner_holes;

  Surface()
  : has_inner_holes(false)
  {}
};

/**
 * @brief Container for object topology extracted from STEP file.
 *
 * Provides read-only access to corners, edges, and surfaces with
 * their connectivity relationships. All elements are 0-indexed.
 *
 * Built by GeometryMapper during STEP file loading.
 */
class Topology
{
public:
  /**
   * @brief Default constructor - creates empty topology
   */
  Topology() = default;

  /**
   * @brief Get corner by ID (const access)
   *
   * @param id Corner ID (0-indexed)
   * @return Reference to Corner struct
   * @throws std::out_of_range if ID is invalid
   */
  const Corner & get_corner(int id) const;

  /**
   * @brief Get edge by ID (const access)
   *
   * @param id Edge ID (0-indexed)
   * @return Reference to Edge struct
   * @throws std::out_of_range if ID is invalid
   */
  const Edge & get_edge(int id) const;

  /**
   * @brief Get surface by ID (const access)
   *
   * @param id Surface ID (0-indexed)
   * @return Reference to Surface struct
   * @throws std::out_of_range if ID is invalid
   */
  const Surface & get_surface(int id) const;

  /**
   * @brief Get all surface IDs
   *
   * @deprecated Use num_surfaces() + index loop or get_all_surfaces() instead.
   * This method allocates a vector of [0..N-1] which is redundant.
   *
   * @return Vector of surface IDs (0-indexed, sorted)
   */
  [[deprecated("Use num_surfaces() + index loop or get_all_surfaces() instead")]]
  std::vector<int> get_all_surface_ids() const;

  /**
   * @brief Get direct access to all corners
   *
   * @return Const reference to corners vector
   */
  const std::vector<Corner> & get_all_corners() const;

  /**
   * @brief Get direct access to all edges
   *
   * @return Const reference to edges vector
   */
  const std::vector<Edge> & get_all_edges() const;

  /**
   * @brief Get direct access to all surfaces (const)
   *
   * @return Const reference to surfaces vector
   */
  const std::vector<Surface> & get_all_surfaces() const;

  /**
   * @brief Get total number of corners
   */
  size_t num_corners() const {return corners_.size();}

  /**
   * @brief Get total number of edges
   */
  size_t num_edges() const {return edges_.size();}

  /**
   * @brief Get total number of surfaces
   */
  size_t num_surfaces() const {return surfaces_.size();}

  /**
   * @brief Add a corner to the topology
   *
   * @param id Sequential ID (0-indexed)
   * @param corner Corner data
   */
  void add_corner(int id, const Corner & corner);

  /**
   * @brief Add an edge to the topology
   *
   * @param id Sequential ID (0-indexed)
   * @param edge Edge data
   */
  void add_edge(int id, const Edge & edge);

  /**
   * @brief Add a surface to the topology
   *
   * @param id Sequential ID (0-indexed)
   * @param surface Surface data
   */
  void add_surface(int id, const Surface & surface);

  /**
   * @brief Clear all topology data
   */
  void clear();

private:
  std::vector<Corner> corners_;
  std::vector<Edge> edges_;
  std::vector<Surface> surfaces_;
};

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__GEOMETRY__TOPOLOGY_HPP_
