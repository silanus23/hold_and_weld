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

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <cmath>

#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

#include "hold_and_weld_gripper_sampler/io/shape_loader.hpp"

using hold_and_weld_gripper_sampler::io::ShapeLoader;  // NOLINT
using hold_and_weld_gripper_sampler::io::ShapeLoaderConfig;  // NOLINT

namespace test_constants
{
constexpr double kPositionTolerance = 1e-6;
constexpr double kCurvedGeometryTolerance = 1e-3;
constexpr double kVolumeTolerance = 1e-9;

constexpr double kDefaultLinearDeflection = 0.0001;
constexpr double kDefaultAngularDeflection = 0.5;

constexpr double kCustomLinearDeflection = 0.001;
constexpr double kCustomAngularDeflection = 0.3;

constexpr double kSmallDimension = 0.1;
constexpr double kMediumDimension = 0.2;
constexpr double kLargeDimension = 0.3;

constexpr double kOffsetX = 1.0;
constexpr double kOffsetY = 2.0;
constexpr double kOffsetZ = 3.0;

constexpr double kTestRadius = 0.05;
constexpr double kTestHeight = 0.1;

constexpr double kGroundPlaneSize = 2.0;
constexpr double kGroundPlaneThickness = 0.01;

constexpr int kBoxFaceCount = 6;
}  // namespace test_constants

class ShapeLoaderTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    loader_ = std::make_unique<ShapeLoader>();
  }

  // Helper to compute bounding box dimensions
  Eigen::Vector3d get_bounding_box_size(const TopoDS_Shape & shape) const
  {
    Bnd_Box box;
    BRepBndLib::Add(shape, box);

    double x_min, y_min, z_min, x_max, y_max, z_max;
    box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

    return Eigen::Vector3d(x_max - x_min, y_max - y_min, z_max - z_min);
  }

  // Helper to compute bounding box center
  Eigen::Vector3d get_bounding_box_center(const TopoDS_Shape & shape) const
  {
    Bnd_Box box;
    BRepBndLib::Add(shape, box);

    double x_min, y_min, z_min, x_max, y_max, z_max;
    box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

    return Eigen::Vector3d(
      (x_min + x_max) / 2.0,
      (y_min + y_max) / 2.0,
      (z_min + z_max) / 2.0);
  }

  // Helper to compute volume
  double get_volume(const TopoDS_Shape & shape) const
  {
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    return props.Mass();
  }

  // Helper to count faces
  int count_faces(const TopoDS_Shape & shape) const
  {
    int count = 0;
    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
      count++;
    }
    return count;
  }

  std::unique_ptr<ShapeLoader> loader_;
};

// Default-constructed loader has expected deflection values and auto-triangulate enabled.
TEST_F(ShapeLoaderTest, DefaultConstructor_HasExpectedConfigValues)
{
  ShapeLoader loader;
  auto config = loader.get_config();

  EXPECT_DOUBLE_EQ(config.linear_deflection, test_constants::kDefaultLinearDeflection);
  EXPECT_DOUBLE_EQ(config.angular_deflection, test_constants::kDefaultAngularDeflection);
  EXPECT_TRUE(config.auto_triangulate);
}

// Custom config is stored and returned verbatim by get_config().
TEST_F(ShapeLoaderTest, CustomConfigConstructor_AppliesProvidedConfig)
{
  ShapeLoaderConfig config;
  config.linear_deflection = test_constants::kCustomLinearDeflection;
  config.angular_deflection = test_constants::kCustomAngularDeflection;
  config.auto_triangulate = false;

  ShapeLoader loader(config);
  auto retrieved_config = loader.get_config();

  EXPECT_DOUBLE_EQ(retrieved_config.linear_deflection, test_constants::kCustomLinearDeflection);
  EXPECT_DOUBLE_EQ(retrieved_config.angular_deflection, test_constants::kCustomAngularDeflection);
  EXPECT_FALSE(retrieved_config.auto_triangulate);
}

// Box bounding box matches requested dimensions; centered at origin; has 6 faces.
TEST_F(ShapeLoaderTest, MakeBox_WithBasicDimensions_CreatesCorrectShape)
{
  Eigen::Vector3d dimensions(
    test_constants::kSmallDimension,
    test_constants::kMediumDimension,
    test_constants::kLargeDimension);
  auto box = loader_->make_box(dimensions);

  EXPECT_FALSE(box.IsNull());

  auto bbox_size = get_bounding_box_size(box);
  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kLargeDimension, test_constants::kPositionTolerance);

  EXPECT_EQ(count_faces(box), test_constants::kBoxFaceCount);

  auto center = get_bounding_box_center(box);
  EXPECT_NEAR(center.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.z(), 0.0, test_constants::kPositionTolerance);
}

// Box volume equals product of its three dimensions.
TEST_F(ShapeLoaderTest, MakeBox_VolumeCalculation_MatchesExpectedVolume)
{
  Eigen::Vector3d dimensions(
    test_constants::kSmallDimension,
    test_constants::kMediumDimension,
    test_constants::kLargeDimension);
  auto box = loader_->make_box(dimensions);

  double expected_volume = test_constants::kSmallDimension *
    test_constants::kMediumDimension *
    test_constants::kLargeDimension;
  double actual_volume = get_volume(box);

  EXPECT_NEAR(actual_volume, expected_volume, test_constants::kVolumeTolerance);
}

// Cylinder bounding box diameter equals 2*radius; height equals requested height.
TEST_F(ShapeLoaderTest, MakeCylinder_WithBasicDimensions_HasCorrectBoundingBox)
{
  auto cylinder = loader_->make_cylinder(test_constants::kTestRadius, test_constants::kTestHeight);

  EXPECT_FALSE(cylinder.IsNull());

  auto bbox_size = get_bounding_box_size(cylinder);
  EXPECT_NEAR(bbox_size.x(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kTestHeight, test_constants::kCurvedGeometryTolerance);
}

// Cylinder volume equals pi*r^2*h.
TEST_F(ShapeLoaderTest, MakeCylinder_VolumeCalculation_MatchesExpectedVolume)
{
  auto cylinder = loader_->make_cylinder(test_constants::kTestRadius, test_constants::kTestHeight);

  double expected_volume = M_PI * test_constants::kTestRadius * test_constants::kTestRadius *
    test_constants::kTestHeight;
  double actual_volume = get_volume(cylinder);

  EXPECT_NEAR(actual_volume, expected_volume, test_constants::kVolumeTolerance);
}

// Sphere bounding box is a cube with side 2*radius.
TEST_F(ShapeLoaderTest, MakeSphere_WithBasicRadius_HasCorrectBoundingBox)
{
  auto sphere = loader_->make_sphere(test_constants::kTestRadius);

  EXPECT_FALSE(sphere.IsNull());

  auto bbox_size = get_bounding_box_size(sphere);
  EXPECT_NEAR(bbox_size.x(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
}

// Sphere volume equals (4/3)*pi*r^3.
TEST_F(ShapeLoaderTest, MakeSphere_VolumeCalculation_MatchesExpectedVolume)
{
  auto sphere = loader_->make_sphere(test_constants::kTestRadius);

  double expected_volume = (4.0 / 3.0) * M_PI * test_constants::kTestRadius *
    test_constants::kTestRadius * test_constants::kTestRadius;
  double actual_volume = get_volume(sphere);

  EXPECT_NEAR(actual_volume, expected_volume, test_constants::kVolumeTolerance);
}

// Ground plane with custom size, position, and thickness matches all three parameters.
TEST_F(ShapeLoaderTest, MakeGroundPlane_WithCustomParams_HasCorrectDimensions)
{
  constexpr double custom_size_x = 3.0;
  constexpr double custom_size_y = 4.0;
  constexpr double custom_z_position = -0.5;
  constexpr double custom_thickness = 0.02;

  auto ground = loader_->make_ground_plane(custom_size_x, custom_size_y, custom_z_position,
    custom_thickness);

  auto bbox_size = get_bounding_box_size(ground);
  EXPECT_NEAR(bbox_size.x(), custom_size_x, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), custom_size_y, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), custom_thickness, test_constants::kPositionTolerance);

  Bnd_Box box;
  BRepBndLib::Add(ground, box);

  double x_min, y_min, z_min, x_max, y_max, z_max;
  box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

  EXPECT_NEAR(z_max, custom_z_position, test_constants::kPositionTolerance);
}

// Combining an empty shape list throws std::runtime_error.
TEST_F(ShapeLoaderTest, CombineShapes_WithEmptyVector_Throws)
{
  std::vector<TopoDS_Shape> empty_shapes;
  EXPECT_THROW(loader_->combine_shapes(empty_shapes), std::runtime_error);
}

// Combining a single shape returns a shape with the same face count.
TEST_F(ShapeLoaderTest, CombineShapes_WithSingleShape_ReturnsSameShape)
{
  auto box = loader_->make_box(Eigen::Vector3d(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));
  std::vector<TopoDS_Shape> shapes = {box};

  auto combined = loader_->combine_shapes(shapes);

  EXPECT_FALSE(combined.IsNull());
  EXPECT_EQ(count_faces(combined), count_faces(box));
}

// Combining three shapes produces a compound whose face count is the sum of the parts.
TEST_F(ShapeLoaderTest, CombineShapes_WithMultipleShapes_CreatesCompoundWithAllFaces)
{
  auto box = loader_->make_box(Eigen::Vector3d(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));
  auto sphere = loader_->make_sphere(test_constants::kTestRadius, Eigen::Vector3d(0.5, 0, 0));
  auto cylinder = loader_->make_cylinder(0.02, test_constants::kTestHeight,
    Eigen::Vector3d(-0.5, 0, 0));

  std::vector<TopoDS_Shape> shapes = {box, sphere, cylinder};
  auto combined = loader_->combine_shapes(shapes);

  EXPECT_FALSE(combined.IsNull());
  EXPECT_EQ(combined.ShapeType(), TopAbs_COMPOUND);

  int box_faces = count_faces(box);
  int sphere_faces = count_faces(sphere);
  int cylinder_faces = count_faces(cylinder);
  int combined_faces = count_faces(combined);

  EXPECT_EQ(combined_faces, box_faces + sphere_faces + cylinder_faces);
}

// Combined rotation and translation correctly affects both extents and center.
TEST_F(ShapeLoaderTest, ApplyTransform_WithCombinedTransform_AppliesBothCorrectly)
{
  auto box = loader_->make_box(Eigen::Vector3d(
    test_constants::kMediumDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));

  Eigen::Vector3d translation(test_constants::kOffsetX, 0.0, 0.0);
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  auto transformed = loader_->apply_transform(box, translation, rotation);

  auto center = get_bounding_box_center(transformed);
  auto bbox_size = get_bounding_box_size(transformed);

  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);

  EXPECT_NEAR(center.x(), test_constants::kOffsetX, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.y(), 0.0, test_constants::kPositionTolerance);
}

// Loading from nonexistent STEP, STL, or URDF paths throws std::runtime_error.
TEST_F(ShapeLoaderTest, LoadFromFile_WithNonexistentFile_ThrowsRuntimeError)
{
  EXPECT_THROW(
    loader_->load_from_step("/nonexistent/path/file.step"),
    std::runtime_error);
  EXPECT_THROW(
    loader_->load_from_stl("/nonexistent/path/file.stl"),
    std::runtime_error);
  EXPECT_THROW(
    loader_->load_from_urdf("/nonexistent/path/file.urdf"),
    std::runtime_error);
}

// URDF sphere geometry produces a shape with bounding box matching 2*radius on all axes.
TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithSphereGeometry_CreatesCorrectShape)
{
  std::string urdf_string =
    R"(
    <robot name="test_robot">
      <link name="base_link">
        <collision>
          <geometry>
            <sphere radius="0.1"/>
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  auto shape = loader_->load_from_urdf_string(urdf_string);

  EXPECT_FALSE(shape.IsNull());

  auto bbox_size = get_bounding_box_size(shape);
  EXPECT_NEAR(bbox_size.x(), test_constants::kMediumDimension,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kMediumDimension,
    test_constants::kCurvedGeometryTolerance);
}

// URDF origin xyz offset is applied and reflected in the bounding box center.
TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithOriginTransform_AppliesTransformCorrectly)
{
  std::string urdf_string =
    R"(
    <robot name="test_robot">
      <link name="base_link">
        <collision>
          <origin xyz="1.0 2.0 3.0" rpy="0 0 0"/>
          <geometry>
            <box size="0.1 0.1 0.1"/>
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  auto shape = loader_->load_from_urdf_string(urdf_string);

  auto center = get_bounding_box_center(shape);
  EXPECT_NEAR(center.x(), test_constants::kOffsetX, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.y(), test_constants::kOffsetY, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.z(), test_constants::kOffsetZ, test_constants::kPositionTolerance);
}

// URDF with multiple links produces a compound with more faces than a single box.
TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithMultipleCollisions_CombinesAllShapes)
{
  std::string urdf_string =
    R"(
    <robot name="test_robot">
      <link name="link1">
        <collision>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
            <box size="0.1 0.1 0.1"/>
          </geometry>
        </collision>
      </link>
      <link name="link2">
        <collision>
          <origin xyz="1 0 0" rpy="0 0 0"/>
          <geometry>
            <sphere radius="0.05"/>
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  auto shape = loader_->load_from_urdf_string(urdf_string);

  EXPECT_FALSE(shape.IsNull());
  EXPECT_EQ(shape.ShapeType(), TopAbs_COMPOUND);

  int face_count = count_faces(shape);
  EXPECT_GT(face_count, test_constants::kBoxFaceCount);
}

// URDF with only visual geometry (no collision element) throws std::runtime_error.
TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithNoCollisionGeometry_ThrowsRuntimeError)
{
  std::string urdf_string =
    R"(
    <robot name="test_robot">
      <link name="base_link">
        <visual>
          <geometry>
            <box size="0.1 0.1 0.1"/>
          </geometry>
        </visual>
      </link>
    </robot>
  )";

  EXPECT_THROW(
    loader_->load_from_urdf_string(urdf_string),
    std::runtime_error);
}

// Malformed XML string throws std::runtime_error.
TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithInvalidXml_ThrowsRuntimeError)
{
  std::string urdf_string = "not valid xml <<<<";

  EXPECT_THROW(
    loader_->load_from_urdf_string(urdf_string),
    std::runtime_error);
}

// XML missing a <robot> root element throws std::runtime_error.
TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithNoRobotElement_ThrowsRuntimeError)
{
  std::string urdf_string =
    R"(
    <notrobot name="test">
      <link name="base_link"/>
    </notrobot>
  )";

  EXPECT_THROW(
    loader_->load_from_urdf_string(urdf_string),
    std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
