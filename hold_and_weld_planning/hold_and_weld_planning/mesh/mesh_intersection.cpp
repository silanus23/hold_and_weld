// Copyright 2026 Berkan Tali
//
// Licensed under the Apache License, Version 2.0 (the 'License');
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an 'AS IS' BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * mesh_intersection.cpp
 *
 * Pybind11 module wrapping CGAL's corefinement to extract the intersection
 * curve between two triangle meshes. Takes two meshes as numpy arrays,
 * inflates mesh_2 slightly to guarantee overlap (converts touching surfaces
 * into actual intersections), and returns the intersection edge segments as
 * an Nx2x3 numpy array.
 *
 * CGAL's exact predicates ensure robust geometric calculations even with
 * near-degenerate inputs.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/transform.h>

#include <vector>
#include <array>
#include <stdexcept>

namespace py = pybind11;
namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;
typedef CGAL::Surface_mesh<Point_3> SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor edge_descriptor;

/**
 * Build a CGAL SurfaceMesh from numpy vertex and face arrays.
 *
 * @param verts  Nx3 float64 array of vertex positions
 * @param faces  Mx3 int32 array of triangle indices
 * @return Populated SurfaceMesh
 * @throws std::invalid_argument if array shapes are invalid
 */
SurfaceMesh build_mesh(
  py::array_t<double> verts,
  py::array_t<int> faces
)
{
  auto v = verts.unchecked<2>();
  auto f = faces.unchecked<2>();

  if (v.shape(1) != 3) {
    throw std::invalid_argument("Vertices must be Nx3 array");
  }
  if (f.shape(1) != 3) {
    throw std::invalid_argument("Faces must be Mx3 array");
  }
  if (v.shape(0) < 3) {
    throw std::invalid_argument("Mesh must have at least 3 vertices");
  }
  if (f.shape(0) < 1) {
    throw std::invalid_argument("Mesh must have at least 1 face");
  }

  SurfaceMesh mesh;

    // Build vertex map: numpy index -> CGAL vertex descriptor
  std::vector<vertex_descriptor> vmap(v.shape(0));
  for (py::ssize_t i = 0; i < v.shape(0); ++i) {
    vmap[i] = mesh.add_vertex(Point_3(v(i, 0), v(i, 1), v(i, 2)));
  }

    // Build triangular faces using vertex indices
  for (py::ssize_t i = 0; i < f.shape(0); ++i) {
    mesh.add_face(vmap[f(i, 0)], vmap[f(i, 1)], vmap[f(i, 2)]);
  }

  return mesh;
}

/**
 * Scale mesh from its centroid by a given factor.
 * Used to create tiny overlap between touching meshes - converts
 * zero-thickness contact into a thin intersection volume that CGAL
 * can detect as a curve.
 *
 * @param mesh    SurfaceMesh to scale in-place
 * @param factor  Scale factor (e.g. 1.002 for 0.2% inflation)
 */
void scale_mesh(SurfaceMesh & mesh, double factor)
{
    // Compute centroid of all vertices
  double cx = 0, cy = 0, cz = 0;
  int n = 0;
  for (auto v : mesh.vertices()) {
    auto & p = mesh.point(v);
    cx += p.x(); cy += p.y(); cz += p.z();
    ++n;
  }

  if (n == 0) {
    return;
  }

  cx /= n; cy /= n; cz /= n;

    // Scale each vertex radially from centroid: p_new = c + factor * (p - c)
  for (auto v : mesh.vertices()) {
    auto & p = mesh.point(v);
    double nx = cx + (p.x() - cx) * factor;
    double ny = cy + (p.y() - cy) * factor;
    double nz = cz + (p.z() - cz) * factor;
    mesh.point(v) = Point_3(nx, ny, nz);
  }
}

/**
 * Extract intersection curve segments between two meshes.
 *
 * Inflates mesh_2 slightly so touching meshes produce an intersection curve,
 * then uses CGAL corefinement to extract that curve as line segments.
 *
 * @param verts1       Nx3 float64 - vertices of mesh 1
 * @param faces1       Mx3 int32   - faces of mesh 1
 * @param verts2       Nx3 float64 - vertices of mesh 2
 * @param faces2       Mx3 int32   - faces of mesh 2
 * @param inflate      Scale factor for mesh_2 inflation (default 1.002 = 0.2%)
 * @return             Px2x3 float64 array of edge segments [[p0, p1], ...]
 *                     Empty array (0,2,3) if no intersection found
 */
py::array_t<double> get_intersection_curve(
  py::array_t<double> verts1,
  py::array_t<int> faces1,
  py::array_t<double> verts2,
  py::array_t<int> faces2,
  double inflate = 1.002
)
{
  SurfaceMesh mesh1 = build_mesh(verts1, faces1);
  SurfaceMesh mesh2 = build_mesh(verts2, faces2);

  scale_mesh(mesh2, inflate);

  auto ecm1 = mesh1.add_property_map<edge_descriptor, bool>(
        "e:is_constrained", false).first;
  auto ecm2 = mesh2.add_property_map<edge_descriptor, bool>(
        "e:is_constrained", false).first;

    // Run CGAL corefinement: splits meshes at intersection and marks intersection edges
    // CGAL's exact predicates ensure robustness even with near-degenerate geometry
  PMP::corefine(
        mesh1, mesh2,
        CGAL::parameters::edge_is_constrained_map(ecm1),
        CGAL::parameters::edge_is_constrained_map(ecm2)
  );

    // Extract constrained edges from mesh1 as line segments
  std::vector<std::array<std::array<double, 3>, 2>> segments;

  for (auto e : mesh1.edges()) {
    if (ecm1[e]) {
      auto h = mesh1.halfedge(e);
      auto & p0 = mesh1.point(mesh1.source(h));
      auto & p1 = mesh1.point(mesh1.target(h));

      segments.push_back({{
          {p0.x(), p0.y(), p0.z()},
          {p1.x(), p1.y(), p1.z()}
        }});
    }
  }

  if (segments.empty()) {
    return py::array_t<double>({0, 2, 3});
  }

    // Pack segments into numpy array with shape (P, 2, 3)
  size_t P = segments.size();
  py::array_t<double> result({(py::ssize_t)P, (py::ssize_t)2, (py::ssize_t)3});
  auto r = result.mutable_unchecked<3>();

  for (size_t i = 0; i < P; ++i) {
    for (int j = 0; j < 2; ++j) {
      for (int k = 0; k < 3; ++k) {
        r(i, j, k) = segments[i][j][k];
      }
    }
  }

  return result;
}

PYBIND11_MODULE(mesh_intersection, m) {
    m.doc() = "CGAL-based mesh-mesh intersection curve extraction";

    m.def(
        "get_intersection_curve",
        &get_intersection_curve,
        py::arg("verts1"),
        py::arg("faces1"),
        py::arg("verts2"),
        py::arg("faces2"),
        py::arg("inflate") = 1.002,
        R"doc(
Extract intersection curve segments between two meshes.

Inflates mesh_2 by `inflate` factor to create tiny overlap for touching
meshes, then uses CGAL corefinement to extract the intersection curve.

Args:
    verts1: (N, 3) float64 array of mesh 1 vertices
    faces1: (M, 3) int32 array of mesh 1 face indices
    verts2: (N, 3) float64 array of mesh 2 vertices
    faces2: (M, 3) int32 array of mesh 2 face indices
    inflate: Scale factor for mesh_2 (default 1.002 = 0.2% inflation)

Returns:
    (P, 2, 3) float64 array of intersection edge segments
        segments[i][0] = start point of segment i
        segments[i][1] = end point of segment i
        )doc"
    );
}
