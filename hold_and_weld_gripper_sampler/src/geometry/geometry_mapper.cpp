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
#include <fstream>
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

    const char * link_name = link->Attribute("name");
    std::string link_name_str = link_name ? link_name : "unknown";
    RCLCPP_DEBUG(logger_, "Processing link: %s", link_name_str.c_str());

    gp_Trsf transform;
    tinyxml2::XMLElement * origin = collision->FirstChildElement("origin");
    if (origin) {
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

      const char * rpy_str = origin->Attribute("rpy");
      if (rpy_str) {
        double roll, pitch, yaw;
        if (std::sscanf(rpy_str, "%lf %lf %lf", &roll, &pitch, &yaw) == 3) {
          gp_Trsf rot_transform;
          rot_transform.SetRotation(rpy_to_quaternion(roll, pitch, yaw));
          transform = transform * rot_transform;
        } else {
          RCLCPP_WARN(logger_, "Failed to parse rpy attribute for link '%s'",
                link_name_str.c_str());
        }
      }
    }

    TopoDS_Shape shape;

    try {
      if (auto * box = geometry->FirstChildElement("box")) {
        double x, y, z;
        if (std::sscanf(box->Attribute("size"), "%lf %lf %lf", &x, &y, &z) != 3) {
          throw std::runtime_error("Invalid box size attributes");
        }
        if (x <= 1e-6 || y <= 1e-6 || z <= 1e-6) {
          throw std::runtime_error("Box dimensions too small");
        }
        // URDF centers box at origin; MakeBox takes a corner, so shift by -half.
        BRepPrimAPI_MakeBox maker(gp_Pnt(-x / 2.0, -y / 2.0, -z / 2.0), x, y, z);
        shape = maker.Shape();

      } else if (auto * cylinder = geometry->FirstChildElement("cylinder")) {
        double r, l;
        if (std::sscanf(cylinder->Attribute("radius"), "%lf", &r) != 1 ||
          std::sscanf(cylinder->Attribute("length"), "%lf", &l) != 1)
        {
          throw std::runtime_error("Invalid cylinder attributes");
        }
        if (r <= 1e-6 || l <= 1e-6) {throw std::runtime_error("Cylinder dimensions too small");}
        // OCCT builds cylinder from z=0 up; URDF centers it, so drop axis origin by half-length.
        BRepPrimAPI_MakeCylinder maker(gp_Ax2(gp_Pnt(0, 0, -l / 2.0), gp_Dir(0, 0, 1)), r, l);
        shape = maker.Shape();

      } else if (auto * sphere = geometry->FirstChildElement("sphere")) {
        double r;
        if (std::sscanf(sphere->Attribute("radius"), "%lf", &r) != 1) {
          throw std::runtime_error("Invalid sphere radius");
        }
        if (r <= 1e-6) {throw std::runtime_error("Sphere radius too small");}
        BRepPrimAPI_MakeSphere maker(r);
        shape = maker.Shape();
      }

      if (!shape.IsNull() && origin) {
        BRepBuilderAPI_Transform transformer(shape, transform, Standard_True);
        if (!transformer.IsDone()) {
          throw std::runtime_error("Geometry transformation failed for link: " + link_name_str);
        }
        shape = transformer.Shape();
      }
    } catch (const Standard_Failure & e) {
      RCLCPP_ERROR(logger_, "OCCT Geometric Error for link '%s': %s", link_name_str.c_str(),
            e.GetMessageString());
      continue;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Error creating geometry for link '%s': %s", link_name_str.c_str(),
            e.what());
      continue;
    }
    builder.Add(compound, shape);
    link_count++;
  }

  RCLCPP_DEBUG(logger_, "Created compound shape from %d link(s)", link_count);
  return compound;
}

TopoDS_Shape GeometryMapper::create_shape_from_urdf_file(const std::string & urdf_path)
{
  RCLCPP_DEBUG(logger_, "Loading URDF file: %s", urdf_path.c_str());

  std::ifstream file(urdf_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open URDF file: " + urdf_path);
  }

  std::ostringstream buffer;
  buffer << file.rdbuf();
  return create_shape_from_urdf_string(buffer.str());
}

TopoDS_Shape GeometryMapper::create_shape_from_step(
  const std::string & step_path,
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation)
{
  RCLCPP_INFO(logger_, "Loading STEP file: %s", step_path.c_str());

  try {
    STEPControl_Reader reader;

    IFSelect_ReturnStatus status = reader.ReadFile(step_path.c_str());
    if (status != IFSelect_RetDone) {
      throw std::runtime_error("Failed to read STEP file: " + step_path);
    }

    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();

    if (shape.IsNull()) {
      throw std::runtime_error("STEP file contains no valid shapes: " + step_path);
    }

    gp_Trsf rot_trsf;
    rot_trsf.SetRotation(gp_Quaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));

    gp_Trsf trans_trsf;
    trans_trsf.SetTranslation(gp_Vec(translation.x(), translation.y(), translation.z()));

    // OCCT applies right-to-left: rotate in local frame first, then translate.
    BRepBuilderAPI_Transform transformer(shape, trans_trsf * rot_trsf, Standard_True);

    if (!transformer.IsDone()) {
      throw std::runtime_error("Failed to apply transform to STEP model");
    }

    TopoDS_Shape result = transformer.Shape();
    if (result.IsNull()) {
      throw std::runtime_error("Transformation resulted in a null shape");
    }

    RCLCPP_INFO(logger_, "STEP file loaded successfully");
    return result;
  } catch (const Standard_Failure & e) {
    throw std::runtime_error(std::string("OCCT STEP Error: ") + e.GetMessageString());
  }
}

Topology GeometryMapper::create_topology_from_shape(
  const TopoDS_Shape & shape,
  const std::string & link_name)
{
  (void)link_name;

  RCLCPP_DEBUG(logger_, "Extracting topology from shape");

  const auto make_fallback_normal = [](const gp_Pnt & center) -> gp_Vec {
      gp_Vec fallback(center.X(), center.Y(), center.Z());
      if (fallback.Magnitude() > 1e-6) {
        fallback.Normalize();
      } else {
        fallback = gp_Vec(0, 0, 1);
      }
      return fallback;
    };

  try {
    TopTools_IndexedMapOfShape vertex_map;
    TopTools_IndexedMapOfShape edge_map;
    face_map_.Clear();

    TopExp::MapShapes(shape, TopAbs_VERTEX, vertex_map);
    TopExp::MapShapes(shape, TopAbs_EDGE, edge_map);
    TopExp::MapShapes(shape, TopAbs_FACE, face_map_);

    RCLCPP_DEBUG(logger_, "Found %d vertices, %d edges, %d faces",
      vertex_map.Extent(), edge_map.Extent(), face_map_.Extent());

    TopTools_IndexedDataMapOfShapeListOfShape vertex_to_edges;
    TopTools_IndexedDataMapOfShapeListOfShape vertex_to_faces;
    TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;

    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertex_to_edges);
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_FACE, vertex_to_faces);
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

    std::vector<Corner> corners(vertex_map.Extent());
    std::vector<Edge> edges(edge_map.Extent());
    std::vector<Surface> surfaces(face_map_.Extent());

    for (int i = 1; i <= vertex_map.Extent(); i++) {
      TopoDS_Vertex vertex = TopoDS::Vertex(vertex_map(i));
      Corner & corner = corners[i - 1];
      corner.position = BRep_Tool::Pnt(vertex);

      if (vertex_to_edges.Contains(vertex)) {
        for (TopTools_ListIteratorOfListOfShape it(vertex_to_edges.FindFromKey(vertex));
          it.More(); it.Next())
        {
          corner.connected_edges.push_back(edge_map.FindIndex(it.Value()) - 1);
        }
      }

      if (vertex_to_faces.Contains(vertex)) {
        for (TopTools_ListIteratorOfListOfShape it(vertex_to_faces.FindFromKey(vertex));
          it.More(); it.Next())
        {
          corner.connected_surfaces.push_back(face_map_.FindIndex(it.Value()) - 1);
        }
      }
    }

    for (int i = 1; i <= edge_map.Extent(); i++) {
      TopoDS_Edge edge = TopoDS::Edge(edge_map(i));
      Edge & edge_data = edges[i - 1];
      edge_data.edge = edge;

      TopoDS_Vertex v1, v2;
      TopExp::Vertices(edge, v1, v2);
      edge_data.corner_ids = std::make_pair(
        vertex_map.FindIndex(v1) - 1,
        vertex_map.FindIndex(v2) - 1);

      if (edge_to_faces.Contains(edge)) {
        for (TopTools_ListIteratorOfListOfShape it(edge_to_faces.FindFromKey(edge));
          it.More(); it.Next())
        {
          edge_data.connected_surfaces.push_back(face_map_.FindIndex(it.Value()) - 1);
        }
      }
    }

    for (int i = 1; i <= face_map_.Extent(); i++) {
      TopoDS_Face face = TopoDS::Face(face_map_(i));
      Surface & surface = surfaces[i - 1];
      surface.face = face;

      for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        surface.edge_ids.push_back(edge_map.FindIndex(TopoDS::Edge(exp.Current())) - 1);
      }

      std::set<int> vertex_set;
      for (TopExp_Explorer exp(face, TopAbs_VERTEX); exp.More(); exp.Next()) {
        vertex_set.insert(vertex_map.FindIndex(TopoDS::Vertex(exp.Current())) - 1);
      }
      surface.corner_ids = std::vector<int>(vertex_set.begin(), vertex_set.end());

      surface.has_inner_holes = has_inner_holes(face);

      GProp_GProps props;
      BRepGProp::SurfaceProperties(face, props);
      surface.center = props.CentreOfMass();

      Handle(Geom_Surface) surf_geom = BRep_Tool::Surface(face);
      if (surf_geom.IsNull()) {
        surface.normal = make_fallback_normal(surface.center);
      } else {
        Standard_Real u_min, u_max, v_min, v_max;
        BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);

        GeomLProp_SLProps props_normal(surf_geom, (u_min + u_max) / 2.0, (v_min + v_max) / 2.0, 1,
          1e-6);

        if (props_normal.IsNormalDefined()) {
          gp_Vec normal = props_normal.Normal();
          if (face.Orientation() == TopAbs_REVERSED) {normal.Reverse();}
          surface.normal = normal;
        } else {
          surface.normal = make_fallback_normal(surface.center);
        }
      }
    }

    Topology topology;
    for (size_t i = 0; i < corners.size(); i++) {
      topology.add_corner(corners[i]);
    }
    for (size_t i = 0; i < edges.size(); i++) {
      topology.add_edge(edges[i]);
    }
    for (size_t i = 0; i < surfaces.size(); i++) {
      topology.add_surface(surfaces[i]);
    }

    RCLCPP_DEBUG(logger_, "Topology extraction complete: %zu corners, %zu edges, %zu surfaces",
      corners.size(), edges.size(), surfaces.size());

    return topology;
  } catch (const Standard_Failure & e) {
    RCLCPP_ERROR(logger_, "OCCT exception during topology extraction: %s", e.GetMessageString());
    throw std::runtime_error(
      std::string("Topology extraction failed: ") + e.GetMessageString());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Exception during topology extraction: %s", e.what());
    throw std::runtime_error(std::string("Topology extraction failed: ") + e.what());
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
  return occt_index - 1;  // OCCT maps are 1-based; topology IDs are 0-based
}

TopoDS_Face GeometryMapper::get_occt_face(int surface_id) const
{
  int occt_index = surface_id + 1;  // 0-based → 1-based
  if (occt_index < 1 || occt_index > face_map_.Extent()) {
    throw std::out_of_range(
      "Surface ID " + std::to_string(surface_id) +
      " out of range [0, " + std::to_string(face_map_.Extent() - 1) + "]");
  }
  return TopoDS::Face(face_map_(occt_index));
}

const TopTools_IndexedMapOfShape & GeometryMapper::get_face_map() const
{
  return face_map_;
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
