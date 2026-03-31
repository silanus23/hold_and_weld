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

#ifndef HOLD_AND_WELD_GRIPPER_SAMPLER__IO__SHAPE_LOADER_HPP_
#define HOLD_AND_WELD_GRIPPER_SAMPLER__IO__SHAPE_LOADER_HPP_

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace io
{

/**
 * @brief Configuration for shape loading
 */
struct ShapeLoaderConfig
{
  double linear_deflection = 0.0001;   // Linear deflection for triangulation [m]
  double angular_deflection = 0.5;     // Angular deflection for triangulation [rad]
  bool auto_triangulate = true;        // Triangulate loaded shapes automatically
};

/**
 * @brief Simple shape loader for secondary objects (fixtures, obstacles, ground)
 *
 * Provides lightweight shape loading WITHOUT topology extraction.
 * Use this for objects that only need collision detection, not grasp sampling.
 * For primary workpieces that need full topology, use GeometryMapper instead.
 *
 * Supported formats: STEP (.step, .stp), STL (.stl), and geometric primitives.
 * All loaded shapes are automatically triangulated for efficient collision detection.
 *
 * @example
 * @code
 * ShapeLoader loader;
 * auto fixture = loader.load_from_step("fixture.step",
 *   Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity());
 * auto ground = loader.make_box(
 *   Eigen::Vector3d(2.0, 2.0, 0.01),
 *   Eigen::Vector3d(0, 0, -0.005),
 *   Eigen::Quaterniond::Identity());
 * @endcode
 */
class ShapeLoader
{
public:
  /** @brief Default constructor with default configuration */
  ShapeLoader();

  /** @brief Constructor with custom configuration */
  explicit ShapeLoader(const ShapeLoaderConfig & config);

  ~ShapeLoader() = default;

  /**
   * @brief Load shape from STEP file
   *
   * @param step_path Path to STEP file (.step or .stp)
   * @param translation Translation to apply [m]
   * @param rotation Rotation to apply
   * @return Loaded and transformed shape
   * @throw std::runtime_error if file cannot be read or is invalid
   */
  TopoDS_Shape load_from_step(
    const std::string & step_path,
    const Eigen::Vector3d & translation = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity());

  /**
   * @brief Load shape from STL file
   *
   * @param stl_path Path to STL file (.stl)
   * @param translation Translation to apply [m]
   * @param rotation Rotation to apply
   * @return Loaded and transformed shape
   * @throw std::runtime_error if file cannot be read or is invalid
   */
  TopoDS_Shape load_from_stl(
    const std::string & stl_path,
    const Eigen::Vector3d & translation = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity());

  /**
   * @brief Load collision geometry from URDF file
   *
   * Extracts all collision geometries and combines them.
   * Does NOT parse kinematics — use GripperParser for that.
   *
   * @param urdf_path Path to URDF file
   * @return Combined collision geometry shape
   * @throw std::runtime_error if file cannot be read or parsed
   */
  TopoDS_Shape load_from_urdf(const std::string & urdf_path);

  /**
   * @brief Load collision geometry from URDF string
   *
   * @param urdf_string URDF XML content as string
   * @return Combined collision geometry shape
   * @throw std::runtime_error if parsing fails
   */
  TopoDS_Shape load_from_urdf_string(const std::string & urdf_string);

  /**
   * @brief Create a box shape
   *
   * @param dimensions Box dimensions (x, y, z) [m]
   * @param center Center position in world frame
   * @param rotation Rotation to apply
   * @return Box shape
   */
  TopoDS_Shape make_box(
    const Eigen::Vector3d & dimensions,
    const Eigen::Vector3d & center = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity());

  /**
   * @brief Create a cylinder shape with axis along Z before rotation
   *
   * @param radius Cylinder radius [m]
   * @param height Cylinder height [m]
   * @param center Center position in world frame
   * @param rotation Rotation to apply
   * @return Cylinder shape
   */
  TopoDS_Shape make_cylinder(
    double radius,
    double height,
    const Eigen::Vector3d & center = Eigen::Vector3d::Zero(),
    const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity());

  /**
   * @brief Create a sphere shape
   *
   * @param radius Sphere radius [m]
   * @param center Center position in world frame
   * @return Sphere shape
   */
  TopoDS_Shape make_sphere(
    double radius,
    const Eigen::Vector3d & center = Eigen::Vector3d::Zero());

  /**
   * @brief Create a ground plane as a thin box at z=0
   *
   * @param size_x Extent in X direction [m]
   * @param size_y Extent in Y direction [m]
   * @param z_position Z position of the ground surface
   * @param thickness Thickness of the ground box [m]
   * @return Ground plane shape
   */
  TopoDS_Shape make_ground_plane(
    double size_x = 2.0,
    double size_y = 2.0,
    double z_position = 0.0,
    double thickness = 0.01);

  /**
   * @brief Triangulate a shape for collision detection
   *
   * Usually not needed if auto_triangulate is enabled in config.
   *
   * @param shape Shape to triangulate (modified in place)
   */
  void triangulate(TopoDS_Shape & shape) const;

  /**
   * @brief Apply transform to a shape
   *
   * @param shape Shape to transform
   * @param translation Translation to apply
   * @param rotation Rotation to apply
   * @return Transformed shape (new copy)
   */
  TopoDS_Shape apply_transform(
    const TopoDS_Shape & shape,
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation) const;

  /**
   * @brief Combine multiple shapes into a compound
   *
   * @param shapes Vector of shapes to combine
   * @return Compound shape containing all input shapes
   */
  TopoDS_Shape combine_shapes(const std::vector<TopoDS_Shape> & shapes) const;

  /** @brief Get current configuration */
  const ShapeLoaderConfig & get_config() const {return config_;}

private:
  ShapeLoaderConfig config_;

  /**
   * @brief Create OCCT transform from Eigen types
   */
  gp_Trsf create_transform(
    const Eigen::Vector3d & translation,
    const Eigen::Quaterniond & rotation) const;

  /**
   * @brief Resolve package:// URLs to absolute paths using ament_index_cpp
   *
   * @param url URL to resolve (package:// or absolute path)
   * @return Resolved absolute path
   * @throw std::runtime_error if package cannot be found
   */
  std::string resolve_package_url(const std::string & url) const;
};

}  // namespace io
}  // namespace hold_and_weld_gripper_sampler

#endif  // HOLD_AND_WELD_GRIPPER_SAMPLER__IO__SHAPE_LOADER_HPP_
