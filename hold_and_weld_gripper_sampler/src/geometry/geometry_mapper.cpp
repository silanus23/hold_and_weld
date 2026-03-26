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

#include "hold_and_weld_gripper_sampler/geometry/geometry_mapper.hpp"

#include <Eigen/Geometry>
#include <tinyxml2.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Surface.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax2.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <rclcpp/rclcpp.hpp>
#include <Standard_Failure.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>

#include "hold_and_weld_gripper_sampler/geometry/occt_utils.hpp"
#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

// Logger for this module
static const rclcpp::Logger logger_ = rclcpp::get_logger("gripper_sampler");

GeometryMapper::GeometryMapper() {}

GeometryMapper::~GeometryMapper() {}


TopoDS_Shape GeometryMapper::create_shape_from_urdf_string(const std::string & urdf_string)
{
  RCLCPP_DEBUG(logger_, "Parsing URDF string for geometry extraction");

  tinyxml2::XMLDocument doc;
  doc.Parse(urdf_string.c_str());

  if (doc.Error()) {
    throw std::runtime_error("Failed to parse URDF string: " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if (!robot) {
    throw std::runtime_error("No <robot> element found in URDF");
  }

  tinyxml2::XMLElement * link = robot->FirstChildElement("link");
  if (!link) {
    throw std::runtime_error("No <link> elements found in URDF");
  }

  TopoDS_Compound compound;
  BRep_Builder builder;
  builder.MakeCompound(compound);
  int link_count = 0;

  for (; link != nullptr; link = link->NextSiblingElement("link")) {
    tinyxml2::XMLElement * collision = link->FirstChildElement("collision");
    if (!collision) {
      continue;
    }

    tinyxml2::XMLElement * geometry = collision->FirstChildElement("geometry");
    if (!geometry) {
      throw std::runtime_error("Link has <collision> but no <geometry> element");
    }

    // Get link name for error messages
    const char * link_name = link->Attribute("name");
    std::string link_name_str = link_name ? link_name : "unknown";
    RCLCPP_DEBUG(logger_, "Processing link: %s", link_name_str.c_str());

    // Parse origin transformation (optional)
    gp_Trsf transform;
    tinyxml2::XMLElement * origin = collision->FirstChildElement("origin");
    if (origin) {
      // Parse xyz translation
      const char * xyz_str = origin->Attribute("xyz");
      if (xyz_str) {
        double x, y, z;
        if (std::sscanf(xyz_str, "%lf %lf %lf", &x, &y, &z) == 3) {
          transform.SetTranslation(gp_Vec(x, y, z));
        } else {
          RCLCPP_WARN(logger_, "Failed to parse xyz attribute for link '%s'",
                link_name_str.c_str());
        }
      }

      // Parse rpy rotation (roll, pitch, yaw)
      const char * rpy_str = origin->Attribute("rpy");
      if (rpy_str) {
        double roll, pitch, yaw;
        if (std::sscanf(rpy_str, "%lf %lf %lf", &roll, &pitch, &yaw) == 3) {
          // Convert RPY (roll-pitch-yaw) to quaternion using ZYX aerospace sequence
          double cy = std::cos(yaw * 0.5);
          double sy = std::sin(yaw * 0.5);
          double cp = std::cos(pitch * 0.5);
          double sp = std::sin(pitch * 0.5);
          double cr = std::cos(roll * 0.5);
          double sr = std::sin(roll * 0.5);

          double qw = cr * cp * cy + sr * sp * sy;
          double qx = sr * cp * cy - cr * sp * sy;
          double qy = cr * sp * cy + sr * cp * sy;
          double qz = cr * cp * sy - sr * sp * cy;

          gp_Quaternion quat(qx, qy, qz, qw);
          gp_Trsf rot_transform;
          rot_transform.SetRotation(quat);
          transform = transform * rot_transform;
        } else {
          RCLCPP_WARN(logger_, "Failed to parse rpy attribute for link '%s'",
                link_name_str.c_str());
        }
      }
    }

    // Check which geometry type
    TopoDS_Shape shape;

    try {
      if (tinyxml2::XMLElement * box = geometry->FirstChildElement("box")) {
        const char * size_str = box->Attribute("size");
        if (!size_str) {
          throw std::runtime_error("Box geometry missing 'size' attribute");
        }

        double x, y, z;
        if (std::sscanf(size_str, "%lf %lf %lf", &x, &y, &z) != 3) {
          throw std::runtime_error("Failed to parse box size");
        }

        // URDF convention: box is centered at origin
        // Create box from corner at (-x/2, -y/2, -z/2) with dimensions (x, y, z)
        gp_Pnt corner(-x / 2.0, -y / 2.0, -z / 2.0);
        shape = BRepPrimAPI_MakeBox(corner, x, y, z).Shape();

      } else if (tinyxml2::XMLElement * cylinder = geometry->FirstChildElement("cylinder")) {
        const char * radius_str = cylinder->Attribute("radius");
        const char * length_str = cylinder->Attribute("length");

        if (!radius_str || !length_str) {
          throw std::runtime_error("Cylinder geometry missing 'radius' or 'length' attribute");
        }

        double radius, length;
        if (std::sscanf(radius_str, "%lf", &radius) != 1 ||
          std::sscanf(length_str, "%lf", &length) != 1)
        {
          throw std::runtime_error("Failed to parse cylinder parameters");
        }

        // URDF convention: cylinder is centered at origin, axis along Z
        // Create cylinder with base at (0, 0, -length/2) extending to (0, 0, +length/2)
        gp_Ax2 axis(gp_Pnt(0, 0, -length / 2.0), gp_Dir(0, 0, 1));
        shape = BRepPrimAPI_MakeCylinder(axis, radius, length).Shape();

      } else if (tinyxml2::XMLElement * sphere = geometry->FirstChildElement("sphere")) {
        const char * radius_str = sphere->Attribute("radius");

        if (!radius_str) {
          throw std::runtime_error("Sphere geometry missing 'radius' attribute");
        }

        double radius;
        if (std::sscanf(radius_str, "%lf", &radius) != 1) {
          throw std::runtime_error("Failed to parse sphere radius");
        }

        shape = BRepPrimAPI_MakeSphere(radius).Shape();

      } else if (geometry->FirstChildElement("mesh")) {
        RCLCPP_WARN(logger_, "Mesh geometry not supported for link '%s', skipping",
          link_name_str.c_str());
        continue;

      } else {
        throw std::runtime_error("Unknown or unsupported geometry type");
      }

      if (origin) {
        BRepBuilderAPI_Transform transformer(shape, transform, true);
        shape = transformer.Shape();
      }
    } catch (const std::exception & e) {
      throw std::runtime_error(
        "Error creating geometry for link '" + link_name_str + "': " + std::string(e.what()));
    } catch (...) {
      throw std::runtime_error(
        "Unknown error creating geometry for link '" + link_name_str + "'");
    }

    builder.Add(compound, shape);
    link_count++;
  }

  RCLCPP_DEBUG(logger_, "Created compound shape from %d link(s)", link_count);
  return compound;
}

TopoDS_Shape GeometryMapper::create_shape_from_urdf_file(const std::string & urdf_path)
{
  RCLCPP_INFO(logger_, "Loading URDF file: %s", urdf_path.c_str());

  tinyxml2::XMLDocument doc;

  if (doc.LoadFile(urdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load URDF file: " + urdf_path +
                           " - " + std::string(doc.ErrorStr()));
  }

  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return create_shape_from_urdf_string(printer.CStr());
}

TopoDS_Shape GeometryMapper::create_shape_from_step(
  const std::string & step_path,
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation)
{
  RCLCPP_INFO(logger_, "Loading STEP file: %s", step_path.c_str());

  try {
    STEPControl_Reader reader;

    // Read the STEP file
    IFSelect_ReturnStatus status = reader.ReadFile(step_path.c_str());
    if (status != IFSelect_RetDone) {
      throw std::runtime_error("Failed to read STEP file: " + step_path);
    }

    // Transfer all shapes from STEP to OCCT
    Standard_Integer num_roots = reader.TransferRoots();
    RCLCPP_DEBUG(logger_, "Transferred %d root(s) from STEP file", num_roots);

    TopoDS_Shape shape = reader.OneShape();

    if (shape.IsNull()) {
      throw std::runtime_error("STEP file contains no valid shapes: " + step_path);
    }

    // Create transformation from translation and rotation
    gp_Trsf transform;

    // Set translation
    transform.SetTranslation(gp_Vec(translation.x(), translation.y(), translation.z()));

    // Set rotation from quaternion
    gp_Quaternion quat(rotation.x(), rotation.y(), rotation.z(), rotation.w());
    gp_Trsf rot_transform;
    rot_transform.SetRotation(quat);

    // Combine transformations: first rotate, then translate
    gp_Trsf combined_transform = transform * rot_transform;

    // Apply transformation to the shape
    BRepBuilderAPI_Transform transformer(shape, combined_transform, true);
    TopoDS_Shape transformed_shape = transformer.Shape();

    RCLCPP_INFO(logger_, "STEP file loaded successfully");
    return transformed_shape;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error loading STEP file '%s': %s",
      step_path.c_str(), e.what());
    throw std::runtime_error(
      "Error loading STEP file '" + step_path + "': " + std::string(e.what()));
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown error loading STEP file '%s'", step_path.c_str());
    throw std::runtime_error("Unknown error loading STEP file '" + step_path + "'");
  }
}

Topology GeometryMapper::create_topology_from_shape(
  const TopoDS_Shape & shape,
  const std::string & link_name)
{
  (void)link_name;  // Reserved for future use (naming/tagging)

  RCLCPP_DEBUG(logger_, "Extracting topology from shape");

  try {
    // Build ID maps for all topological elements
    TopTools_IndexedMapOfShape vertex_map;
    TopTools_IndexedMapOfShape edge_map;
    face_map_.Clear();

    TopExp::MapShapes(shape, TopAbs_VERTEX, vertex_map);
    TopExp::MapShapes(shape, TopAbs_EDGE, edge_map);
    TopExp::MapShapes(shape, TopAbs_FACE, face_map_);

    RCLCPP_DEBUG(logger_, "Found %d vertices, %d edges, %d faces",
    vertex_map.Extent(), edge_map.Extent(), face_map_.Extent());

  // Build ancestor maps for connectivity
    TopTools_IndexedDataMapOfShapeListOfShape vertex_to_edges;
    TopTools_IndexedDataMapOfShapeListOfShape vertex_to_faces;
    TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;

    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertex_to_edges);
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_FACE, vertex_to_faces);
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

  // Initialize storage vectors
    std::vector<Corner> corners(vertex_map.Extent());
    std::vector<Edge> edges(edge_map.Extent());
    std::vector<Surface> surfaces(face_map_.Extent());

    for (int i = 1; i <= vertex_map.Extent(); i++) {
      TopoDS_Vertex vertex = TopoDS::Vertex(vertex_map(i));
      int corner_id = i - 1;  // Convert to 0-based indexing

      Corner & corner = corners[corner_id];
      corner.position = BRep_Tool::Pnt(vertex);

    // Find connected edges
      if (vertex_to_edges.Contains(vertex)) {
        const TopTools_ListOfShape & edge_list = vertex_to_edges.FindFromKey(vertex);
        for (TopTools_ListIteratorOfListOfShape it(edge_list); it.More(); it.Next()) {
          int edge_id = edge_map.FindIndex(it.Value()) - 1;
          corner.connected_edges.push_back(edge_id);
        }
      }

    // Find connected surfaces
      if (vertex_to_faces.Contains(vertex)) {
        const TopTools_ListOfShape & face_list = vertex_to_faces.FindFromKey(vertex);
        for (TopTools_ListIteratorOfListOfShape it(face_list); it.More(); it.Next()) {
          int face_id = face_map_.FindIndex(it.Value()) - 1;
          corner.connected_surfaces.push_back(face_id);
        }
      }
    }

    for (int i = 1; i <= edge_map.Extent(); i++) {
      TopoDS_Edge edge = TopoDS::Edge(edge_map(i));
      int edge_id = i - 1;  // Convert to 0-based indexing

      Edge & edge_data = edges[edge_id];

      edge_data.edge = edge;

      TopoDS_Vertex v1, v2;
      TopExp::Vertices(edge, v1, v2);
      int v1_id = vertex_map.FindIndex(v1) - 1;
      int v2_id = vertex_map.FindIndex(v2) - 1;
      edge_data.corner_ids = std::make_pair(v1_id, v2_id);

    // Classify edge type (LINE, CIRCLE, or SPLINE)
      edge_data.type = classify_edge(edge);

    // Find connected surfaces
      if (edge_to_faces.Contains(edge)) {
        const TopTools_ListOfShape & face_list = edge_to_faces.FindFromKey(edge);
        for (TopTools_ListIteratorOfListOfShape it(face_list); it.More(); it.Next()) {
          int face_id = face_map_.FindIndex(it.Value()) - 1;
          edge_data.connected_surfaces.push_back(face_id);
        }
      }
    }

    for (int i = 1; i <= face_map_.Extent(); i++) {
      TopoDS_Face face = TopoDS::Face(face_map_(i));
      int face_id = i - 1;  // Convert to 0-based indexing

      Surface & surface = surfaces[face_id];
      surface.face = face;

    // Extract edges of this face
      TopExp_Explorer edge_explorer(face, TopAbs_EDGE);
      for (; edge_explorer.More(); edge_explorer.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());
        int edge_id = edge_map.FindIndex(edge) - 1;
        surface.edge_ids.push_back(edge_id);
      }

    // Extract vertices of this face (use set to avoid duplicates)
      std::set<int> vertex_set;
      TopExp_Explorer vertex_explorer(face, TopAbs_VERTEX);
      for (; vertex_explorer.More(); vertex_explorer.Next()) {
        TopoDS_Vertex vertex = TopoDS::Vertex(vertex_explorer.Current());
        int vertex_id = vertex_map.FindIndex(vertex) - 1;
        vertex_set.insert(vertex_id);
      }
      surface.corner_ids = std::vector<int>(vertex_set.begin(), vertex_set.end());

    // Check if surface has inner holes
      surface.has_inner_holes = has_inner_holes(face);

      // Compute surface center (centroid)
      GProp_GProps props;
      BRepGProp::SurfaceProperties(face, props);
      surface.center = props.CentreOfMass();

      // Compute surface normal
      try {
        Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
        Standard_Real u_min, u_max, v_min, v_max;
        BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
        Standard_Real u_mid = (u_min + u_max) / 2.0;
        Standard_Real v_mid = (v_min + v_max) / 2.0;

        GeomLProp_SLProps props_normal(surf, u_mid, v_mid, 1, 1e-6);
        if (props_normal.IsNormalDefined()) {
          gp_Vec normal = props_normal.Normal();

          // Reversed faces have inverted normals - correct orientation
          if (face.Orientation() == TopAbs_REVERSED) {
            normal.Reverse();
          }
          surface.normal = normal;
        } else {
          gp_Vec fallback(surface.center.X(), surface.center.Y(), surface.center.Z());
          if (fallback.Magnitude() > 1e-9) {
            fallback.Normalize();
          } else {
            fallback = gp_Vec(0, 0, 1);
          }
          surface.normal = fallback;
        }
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(logger_, "Normal computation failed for face %d: %s - using fallback",
          face_id, e.what());
        gp_Vec fallback(surface.center.X(), surface.center.Y(), surface.center.Z());
        if (fallback.Magnitude() > 1e-9) {
          fallback.Normalize();
        } else {
          fallback = gp_Vec(0, 0, 1);
        }
        surface.normal = fallback;
      } catch (...) {
        RCLCPP_DEBUG(logger_,
              "Normal computation failed for face %d (unknown error) - using fallback",
          face_id);
        gp_Vec fallback(surface.center.X(), surface.center.Y(), surface.center.Z());
        if (fallback.Magnitude() > 1e-9) {
          fallback.Normalize();
        } else {
          fallback = gp_Vec(0, 0, 1);
        }
        surface.normal = fallback;
      }
    }

    Topology topology;

    for (size_t i = 0; i < corners.size(); i++) {
      topology.add_corner(i, corners[i]);
    }

    for (size_t i = 0; i < edges.size(); i++) {
      topology.add_edge(i, edges[i]);
    }

    for (size_t i = 0; i < surfaces.size(); i++) {
      topology.add_surface(i, surfaces[i]);
    }

    RCLCPP_DEBUG(logger_, "Topology extraction complete: %zu corners, %zu edges, %zu surfaces",
        corners.size(), edges.size(), surfaces.size());

    return topology;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during topology extraction: %s", e.what());
    throw std::runtime_error("Topology extraction failed: " + std::string(e.what()));
  } catch (...) {
    RCLCPP_ERROR(logger_, "Unknown exception during topology extraction");
    throw std::runtime_error("Topology extraction failed: unknown error");
  }
}

Topology GeometryMapper::load_from_urdf_string(const std::string & urdf_string)
{
  TopoDS_Shape shape = create_shape_from_urdf_string(urdf_string);
  return create_topology_from_shape(shape, "urdf_combined");
}

Topology GeometryMapper::load_from_urdf_file(const std::string & urdf_path)
{
  TopoDS_Shape shape = create_shape_from_urdf_file(urdf_path);
  return create_topology_from_shape(shape, "urdf_combined");
}

Topology GeometryMapper::load_from_step(
  const std::string & step_path,
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation)
{
  TopoDS_Shape shape = create_shape_from_step(step_path, translation, rotation);
  return create_topology_from_shape(shape, "step_model");
}

Topology GeometryMapper::load_from_shape(
  const TopoDS_Shape & shape,
  const std::string & name)
{
  return create_topology_from_shape(shape, name);
}

int GeometryMapper::find_topology_surface_id(const TopoDS_Face & occt_face) const
{
  int occt_index = face_map_.FindIndex(occt_face);

  if (occt_index == 0) {
    throw std::runtime_error("Face not found in topology");
  }

  return occt_index - 1;
}

const TopTools_IndexedMapOfShape & GeometryMapper::get_face_map() const
{
  return face_map_;
}

TopoDS_Face GeometryMapper::get_occt_face(int surface_id) const
{
  int occt_index = surface_id + 1;

  if (occt_index < 1 || occt_index > face_map_.Extent()) {
    throw std::out_of_range(
      "Surface ID " + std::to_string(surface_id) +
      " out of range [0, " + std::to_string(face_map_.Extent() - 1) + "]"
    );
  }

  return TopoDS::Face(face_map_(occt_index));
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
