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
 * Extracts unordered contact-boundary seed points between two meshes.
 *
 * A face of mesh_1 is "in contact" when any of its vertices or its centroid
 * lies within epsilon of mesh_2. The seed points are the unique vertices of
 * the contact strip's boundary halfedges (edges whose neighbouring face is
 * outside the contact strip) — these sit on the contact perimeter, near the
 * true weld edge. Ordering, chaining, and edge refinement are deliberately
 * NOT done here; the Python field pipeline owns those.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace py = pybind11;
namespace PMP = CGAL::Polygon_mesh_processing;

typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef K::Point_3                                            Point_3;
typedef K::Vector_3                                           Vector_3;
typedef CGAL::Surface_mesh<Point_3>                           SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor     face_descriptor;
typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor   vertex_descriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor halfedge_descriptor;

typedef CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh> Primitive;
typedef CGAL::AABB_traits<K, Primitive>                       AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits>                          AABB_Tree;


/** Build a CGAL SurfaceMesh from numpy vertex and face arrays. */
SurfaceMesh build_mesh(
    py::array_t<double> verts,
    py::array_t<int>    faces)
{
    auto v = verts.unchecked<2>();
    auto f = faces.unchecked<2>();

    if (v.shape(1) != 3)
        throw std::invalid_argument("Vertices must be Nx3");
    if (f.shape(1) != 3)
        throw std::invalid_argument("Faces must be Mx3");
    if (v.shape(0) < 3)
        throw std::invalid_argument("Mesh must have at least 3 vertices");
    if (f.shape(0) < 1)
        throw std::invalid_argument("Mesh must have at least 1 face");

    SurfaceMesh mesh;
    std::vector<vertex_descriptor> vmap(v.shape(0));

    for (py::ssize_t i = 0; i < v.shape(0); ++i)
        vmap[i] = mesh.add_vertex(Point_3(v(i,0), v(i,1), v(i,2)));

    for (py::ssize_t i = 0; i < f.shape(0); ++i) {
        face_descriptor fd = mesh.add_face(vmap[f(i,0)], vmap[f(i,1)], vmap[f(i,2)]);
        if (fd == SurfaceMesh::null_face())
            throw std::invalid_argument(
                "Non-manifold or degenerate face at index " + std::to_string(i));
    }

    return mesh;
}


/**
 * Extract unordered contact-boundary seed points of mesh_1 relative to mesh_2.
 *
 * Returns a Python dict with:
 *   'points':  (N,3) float64 - unique boundary vertex positions (unordered)
 *   'normals': (N,3) float64 - averaged contact-face normal per vertex
 */
py::dict get_contact_points(
    py::array_t<double> verts1,
    py::array_t<int>    faces1,
    py::array_t<double> verts2,
    py::array_t<int>    faces2,
    double              epsilon = 1e-3)
{
    SurfaceMesh mesh1 = build_mesh(verts1, faces1);
    SurfaceMesh mesh2 = build_mesh(verts2, faces2);

    AABB_Tree tree(faces(mesh2).first, faces(mesh2).second, mesh2);
    tree.accelerate_distance_queries();

    const double sq_epsilon = epsilon * epsilon;

    auto face_normals = mesh1.add_property_map<face_descriptor, Vector_3>(
        "f:normal", CGAL::NULL_VECTOR).first;
    PMP::compute_face_normals(mesh1, face_normals);

    // Contact strip: faces with any vertex or the centroid within epsilon of
    // mesh_2. Testing all four points (not just the centroid) keeps large
    // triangles that touch mesh_2 only near a corner in the strip.
    std::unordered_set<size_t> contact_set;

    for (face_descriptor fd : mesh1.faces()) {
        halfedge_descriptor h = mesh1.halfedge(fd);
        Point_3 p0 = mesh1.point(mesh1.source(h));
        Point_3 p1 = mesh1.point(mesh1.target(h));
        Point_3 p2 = mesh1.point(mesh1.target(mesh1.next(h)));
        Point_3 centroid(
            (p0.x()+p1.x()+p2.x())/3.0,
            (p0.y()+p1.y()+p2.y())/3.0,
            (p0.z()+p1.z()+p2.z())/3.0
        );

        for (const Point_3& q : {p0, p1, p2, centroid}) {
            if (tree.squared_distance(q) <= sq_epsilon) {
                contact_set.insert(static_cast<size_t>(fd));
                break;
            }
        }
    }

    // Seed points: vertices of boundary halfedges — halfedges of a contact
    // face whose opposite face is missing or outside the contact strip.
    // Normals are accumulated from the emitting contact faces and averaged.
    std::unordered_map<size_t, Vector_3> seed_normal_sum;
    std::unordered_map<size_t, Point_3>  seed_position;

    for (size_t idx : contact_set) {
        face_descriptor fd(static_cast<SurfaceMesh::size_type>(idx));

        halfedge_descriptor h = mesh1.halfedge(fd);
        halfedge_descriptor start = h;
        do {
            face_descriptor nb = mesh1.face(mesh1.opposite(h));
            bool nb_in_contact = (
                nb != SurfaceMesh::null_face() &&
                contact_set.count(static_cast<size_t>(nb))
            );

            if (!nb_in_contact) {
                for (vertex_descriptor v : {mesh1.source(h), mesh1.target(h)}) {
                    size_t vi = static_cast<size_t>(v);
                    seed_position[vi] = mesh1.point(v);
                    auto it = seed_normal_sum.find(vi);
                    if (it == seed_normal_sum.end())
                        seed_normal_sum[vi] = face_normals[fd];
                    else
                        it->second = it->second + face_normals[fd];
                }
            }
            h = mesh1.next(h);
        } while (h != start);
    }

    size_t N = seed_position.size();
    py::array_t<double> np_points ({(py::ssize_t)N, (py::ssize_t)3});
    py::array_t<double> np_normals({(py::ssize_t)N, (py::ssize_t)3});

    auto pv = np_points.mutable_unchecked<2>();
    auto nv = np_normals.mutable_unchecked<2>();

    size_t i = 0;
    for (const auto& [vi, p] : seed_position) {
        pv(i,0) = p.x();
        pv(i,1) = p.y();
        pv(i,2) = p.z();

        Vector_3 n = seed_normal_sum.at(vi);
        double norm = std::sqrt(n.x()*n.x() + n.y()*n.y() + n.z()*n.z());
        if (norm > 1e-10)
            n = Vector_3(n.x()/norm, n.y()/norm, n.z()/norm);
        else
            n = Vector_3(0, 0, 1);

        nv(i,0) = n.x();
        nv(i,1) = n.y();
        nv(i,2) = n.z();
        ++i;
    }

    py::dict result;
    result["points"]  = np_points;
    result["normals"] = np_normals;
    return result;
}


PYBIND11_MODULE(mesh_intersection, m) {
    m.doc() = "CGAL-based contact-boundary seed point extraction between meshes";

    m.def(
        "get_contact_points",
        &get_contact_points,
        py::arg("verts1"),
        py::arg("faces1"),
        py::arg("verts2"),
        py::arg("faces2"),
        py::arg("epsilon") = 1e-3,
        R"doc(
Extract unordered contact-boundary seed points of mesh_1 relative to mesh_2.
Returns a dict with 'points' (N,3) and 'normals' (N,3). Points are the unique
vertices of the contact strip's boundary halfedges, unordered.
)doc"
    );
}
