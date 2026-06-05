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
 * Extracts ordered seam vertices from the contact chain between two meshes,
 * grouped into separate seams via face-adjacency decomposition.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <utility>
#include <stdexcept>
#include <iostream>
#include <cmath>

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
typedef CGAL::AABB_traits_3<K, Primitive>                     AABB_Traits;
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

    for (py::ssize_t i = 0; i < f.shape(0); ++i)
        mesh.add_face(vmap[f(i,0)], vmap[f(i,1)], vmap[f(i,2)]);

    return mesh;
}


/** Compute mesh centroid as the average of all vertex positions. */
Point_3 compute_mesh_centroid(const SurfaceMesh& mesh)
{
    double cx = 0, cy = 0, cz = 0;
    int n = 0;
    for (vertex_descriptor v : mesh.vertices()) {
        const Point_3& p = mesh.point(v);
        cx += p.x(); cy += p.y(); cz += p.z();
        ++n;
    }
    if (n == 0) return Point_3(0, 0, 0);
    return Point_3(cx/n, cy/n, cz/n);
}


/** Squared Euclidean distance between two Point_3. */
double sq_dist(const Point_3& a, const Point_3& b)
{
    double dx = a.x()-b.x(), dy = a.y()-b.y(), dz = a.z()-b.z();
    return dx*dx + dy*dy + dz*dz;
}


/** Return the three halfedges of a triangular face. */
std::vector<halfedge_descriptor> get_face_halfedges(
    const SurfaceMesh& mesh,
    face_descriptor    fd)
{
    std::vector<halfedge_descriptor> halfedges;
    halfedge_descriptor h = mesh.halfedge(fd);
    halfedge_descriptor start = h;
    do {
        halfedges.push_back(h);
        h = mesh.next(h);
    } while (h != start);
    return halfedges;
}


/**
 * Select the seam vertex from an outer-chain face based on boundary edge count.
 * Returns false (skip face) for 0 or 3 boundary edges.
 */
bool select_seam_vertex(
    const SurfaceMesh&                      mesh,
    face_descriptor                         fd,
    const std::vector<halfedge_descriptor>& boundary_halfedges,
    const Point_3&                          mesh_centroid,
    Point_3&                                out_vertex)
{
    int num_boundary = static_cast<int>(boundary_halfedges.size());

    if (num_boundary == 0 || num_boundary == 3)
        return false;

    if (num_boundary == 1) {
        std::vector<halfedge_descriptor> all_he = get_face_halfedges(mesh, fd);

        std::vector<halfedge_descriptor> non_boundary;
        for (auto& h : all_he) {
            bool is_boundary = false;
            for (auto& bh : boundary_halfedges) {
                if (h == bh) { is_boundary = true; break; }
            }
            if (!is_boundary) non_boundary.push_back(h);
        }

        vertex_descriptor v0s = mesh.source(non_boundary[0]);
        vertex_descriptor v0t = mesh.target(non_boundary[0]);
        vertex_descriptor v1s = mesh.source(non_boundary[1]);
        vertex_descriptor v1t = mesh.target(non_boundary[1]);

        vertex_descriptor shared = SurfaceMesh::null_vertex();
        if      (v0s == v1s || v0s == v1t) shared = v0s;
        else if (v0t == v1s || v0t == v1t) shared = v0t;

        if (shared == SurfaceMesh::null_vertex()) {
            return false;
        }

        out_vertex = mesh.point(shared);
        return true;
    }

    if (num_boundary == 2) {
        vertex_descriptor bv0s = mesh.source(boundary_halfedges[0]);
        vertex_descriptor bv0t = mesh.target(boundary_halfedges[0]);
        vertex_descriptor bv1s = mesh.source(boundary_halfedges[1]);
        vertex_descriptor bv1t = mesh.target(boundary_halfedges[1]);

        vertex_descriptor corner = SurfaceMesh::null_vertex();
        if      (bv0s == bv1s || bv0s == bv1t) corner = bv0s;
        else if (bv0t == bv1s || bv0t == bv1t) corner = bv0t;

        std::vector<vertex_descriptor> candidates;
        for (vertex_descriptor v : {bv0s, bv0t, bv1s, bv1t}) {
            if (v != corner) candidates.push_back(v);
        }

        if (candidates.empty()) {
            return false;
        }

        vertex_descriptor best = candidates[0];
        double best_dist = sq_dist(mesh.point(candidates[0]), mesh_centroid);

        for (size_t i = 1; i < candidates.size(); ++i) {
            double d = sq_dist(mesh.point(candidates[i]), mesh_centroid);
            if (d > best_dist) {
                best_dist = d;
                best = candidates[i];
            }
        }

        out_vertex = mesh.point(best);
        return true;
    }

    return false;
}


/** Seam vertex and face normal for one emitted outer-chain face. */
struct EmittedFace {
    Point_3  vertex;
    Vector_3 normal;
};

/** Ordered sequence of emitted-face indices forming one seam. */
struct SeamPath {
    std::vector<size_t> faces;
    bool                is_closed;
};

/** Normalize an undirected face-pair edge to (min, max). */
static std::pair<size_t, size_t> make_edge(size_t a, size_t b)
{
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

/**
 * Decompose the emitted-face adjacency graph into ordered seams.
 * Produces maximal non-branching paths from tips/forks, then sweeps
 * remaining edges as closed cycles.
 */
std::vector<SeamPath> decompose_into_seams(
    const std::unordered_map<size_t, std::vector<size_t>>& adj)
{
    std::vector<SeamPath> seams;
    std::set<std::pair<size_t, size_t>> used_edges;

    auto next_unused = [&](size_t cur) -> long long {
        auto it = adj.find(cur);
        if (it == adj.end()) return -1;
        for (size_t nb : it->second) {
            if (!used_edges.count(make_edge(cur, nb)))
                return static_cast<long long>(nb);
        }
        return -1;
    };

    for (const auto& kv : adj) {
        size_t seed = kv.first;
        if (kv.second.size() == 2) continue;

        for (size_t first_nb : kv.second) {
            auto e = make_edge(seed, first_nb);
            if (used_edges.count(e)) continue;

            used_edges.insert(e);

            SeamPath path;
            path.is_closed = false;
            path.faces.push_back(seed);
            path.faces.push_back(first_nb);

            size_t cur = first_nb;
            while (true) {
                auto cit = adj.find(cur);
                if (cit == adj.end() || cit->second.size() != 2) break;

                long long nx = next_unused(cur);
                if (nx < 0) break;

                used_edges.insert(make_edge(cur, static_cast<size_t>(nx)));
                path.faces.push_back(static_cast<size_t>(nx));
                cur = static_cast<size_t>(nx);
            }

            seams.push_back(std::move(path));
        }
    }

    for (const auto& kv : adj) {
        size_t start = kv.first;

        for (size_t first_nb : kv.second) {
            auto e = make_edge(start, first_nb);
            if (used_edges.count(e)) continue;

            used_edges.insert(e);

            SeamPath path;
            path.is_closed = true;
            path.faces.push_back(start);
            path.faces.push_back(first_nb);

            size_t cur = first_nb;
            while (cur != start) {
                long long nx = next_unused(cur);
                if (nx < 0) break;

                used_edges.insert(make_edge(cur, static_cast<size_t>(nx)));
                path.faces.push_back(static_cast<size_t>(nx));
                cur = static_cast<size_t>(nx);
            }

            seams.push_back(std::move(path));
        }
    }

    return seams;
}


/**
 * Extract ordered seam vertices from mesh_1 relative to mesh_2.
 *
 * Returns a Python list of dicts, each with:
 *   'vertices':  (K,3) float64 - ordered seam vertex positions
 *   'normals':   (K,3) float64 - aligned face normal at each vertex
 *   'is_closed': bool           - whether the seam forms a closed loop
 */
py::list get_seam_vertices(
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

    Point_3 mesh1_centroid = compute_mesh_centroid(mesh1);

    std::unordered_map<size_t, face_descriptor> proximity_set;

    double min_sq_dist = std::numeric_limits<double>::max();
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

        double sq_d = tree.squared_distance(centroid);
        if (sq_d < min_sq_dist) min_sq_dist = sq_d;

        if (sq_d <= sq_epsilon) {
            proximity_set[static_cast<size_t>(fd)] = fd;
        }
    }

    std::unordered_map<size_t, EmittedFace> emitted;
    std::unordered_map<size_t, face_descriptor> emitted_fd;

    for (auto& [idx, fd] : proximity_set) {
        std::vector<halfedge_descriptor> all_he = get_face_halfedges(mesh1, fd);

        std::vector<halfedge_descriptor> boundary_halfedges;

        for (halfedge_descriptor h : all_he) {
            halfedge_descriptor opp = mesh1.opposite(h);
            face_descriptor nb = mesh1.face(opp);

            bool nb_in_proximity = (
                nb != SurfaceMesh::null_face() &&
                proximity_set.count(static_cast<size_t>(nb))
            );

            if (!nb_in_proximity)
                boundary_halfedges.push_back(h);
        }

        if (boundary_halfedges.size() == 0)
            continue;

        Point_3 seam_vertex;
        bool ok = select_seam_vertex(
            mesh1, fd, boundary_halfedges, mesh1_centroid, seam_vertex
        );

        if (!ok) continue;

        Vector_3 n = face_normals[fd];
        double norm = std::sqrt(n.x()*n.x() + n.y()*n.y() + n.z()*n.z());
        if (norm > 1e-10)
            n = Vector_3(n.x()/norm, n.y()/norm, n.z()/norm);
        else
            n = Vector_3(0, 0, 1);

        emitted[idx]    = EmittedFace{seam_vertex, n};
        emitted_fd[idx] = fd;
    }

    std::unordered_map<size_t, std::vector<size_t>> adj;
    adj.reserve(emitted.size());

    for (auto& [idx, fd] : emitted_fd) {
        std::vector<size_t>& neighbours = adj[idx];
        std::vector<halfedge_descriptor> all_he = get_face_halfedges(mesh1, fd);

        for (halfedge_descriptor h : all_he) {
            halfedge_descriptor opp = mesh1.opposite(h);
            face_descriptor nb = mesh1.face(opp);
            if (nb == SurfaceMesh::null_face()) continue;

            size_t nb_idx = static_cast<size_t>(nb);
            if (emitted.count(nb_idx))
                neighbours.push_back(nb_idx);
        }
    }

    std::vector<SeamPath> seam_paths = decompose_into_seams(adj);

    const double dup_sq_tol = 1e-9 * 1e-9;
    py::list result;

    for (const SeamPath& sp : seam_paths) {
        std::vector<Point_3>  verts;
        std::vector<Vector_3> norms;

        for (size_t idx : sp.faces) {
            const EmittedFace& ef = emitted.at(idx);
            if (!verts.empty() && sq_dist(verts.back(), ef.vertex) < dup_sq_tol)
                continue;
            verts.push_back(ef.vertex);
            norms.push_back(ef.normal);
        }

        if (verts.size() < 2) continue;

        size_t N = verts.size();
        py::array_t<double> np_vertices({(py::ssize_t)N, (py::ssize_t)3});
        py::array_t<double> np_normals ({(py::ssize_t)N, (py::ssize_t)3});

        auto v = np_vertices.mutable_unchecked<2>();
        auto n = np_normals.mutable_unchecked<2>();

        for (size_t i = 0; i < N; ++i) {
            v(i,0) = verts[i].x();
            v(i,1) = verts[i].y();
            v(i,2) = verts[i].z();

            n(i,0) = norms[i].x();
            n(i,1) = norms[i].y();
            n(i,2) = norms[i].z();
        }

        py::dict seam;
        seam["vertices"]  = np_vertices;
        seam["normals"]   = np_normals;
        seam["is_closed"] = sp.is_closed;
        result.append(seam);
    }

    return result;
}


PYBIND11_MODULE(mesh_intersection, m) {
    m.doc() = "CGAL-based ordered seam vertex extraction from mesh contact zones";

    m.def(
        "get_seam_vertices",
        &get_seam_vertices,
        py::arg("verts1"),
        py::arg("faces1"),
        py::arg("verts2"),
        py::arg("faces2"),
        py::arg("epsilon") = 1e-3,
        R"doc(
Extract ordered seam vertices from mesh_1 relative to mesh_2, grouped into seams.
Each returned dict has 'vertices' (K,3), 'normals' (K,3), and 'is_closed' (bool).
)doc"
    );
}
