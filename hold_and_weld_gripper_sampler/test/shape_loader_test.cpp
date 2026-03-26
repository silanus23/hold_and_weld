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

TEST_F(ShapeLoaderTest, DefaultConstructor_HasExpectedConfigValues)
{
  ShapeLoader loader;
  auto config = loader.get_config();

  EXPECT_DOUBLE_EQ(config.linear_deflection, test_constants::kDefaultLinearDeflection);
  EXPECT_DOUBLE_EQ(config.angular_deflection, test_constants::kDefaultAngularDeflection);
  EXPECT_TRUE(config.auto_triangulate);
}

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

TEST_F(ShapeLoaderTest, MakeBox_WithBasicDimensions_CreatesCorrectShape)
{
  Eigen::Vector3d dimensions(
    test_constants::kSmallDimension,
    test_constants::kMediumDimension,
    test_constants::kLargeDimension);
  auto box = loader_->make_box(dimensions);

  EXPECT_FALSE(box.IsNull());

  // Check dimensions via bounding box
  auto bbox_size = get_bounding_box_size(box);
  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kLargeDimension, test_constants::kPositionTolerance);

  // Box should have 6 faces
  EXPECT_EQ(count_faces(box), test_constants::kBoxFaceCount);
}

TEST_F(ShapeLoaderTest, MakeBox_WithDefaultCenter_IsCenteredAtOrigin)
{
  Eigen::Vector3d dimensions(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension);
  auto box = loader_->make_box(dimensions);

  auto center = get_bounding_box_center(box);
  EXPECT_NEAR(center.x(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.y(), 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.z(), 0.0, test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, MakeBox_WithTranslation_IsCenteredAtSpecifiedPosition)
{
  Eigen::Vector3d dimensions(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension);
  Eigen::Vector3d center(
    test_constants::kOffsetX,
    test_constants::kOffsetY,
    test_constants::kOffsetZ);

  auto box = loader_->make_box(dimensions, center);

  auto bbox_center = get_bounding_box_center(box);
  EXPECT_NEAR(bbox_center.x(), test_constants::kOffsetX, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_center.y(), test_constants::kOffsetY, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_center.z(), test_constants::kOffsetZ, test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, MakeBox_WithRotation90DegreesAroundZ_SwapsXAndYDimensions)
{
  Eigen::Vector3d dimensions(
    test_constants::kMediumDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension);  // Longer in X
  Eigen::Vector3d center = Eigen::Vector3d::Zero();

  // Rotate 90 degrees around Z axis - X becomes Y
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));

  auto box = loader_->make_box(dimensions, center, rotation);

  auto bbox_size = get_bounding_box_size(box);

  // After rotation, longer dimension should be in Y
  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
}

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

TEST_F(ShapeLoaderTest, MakeCylinder_WithBasicDimensions_HasCorrectBoundingBox)
{
  auto cylinder = loader_->make_cylinder(test_constants::kTestRadius, test_constants::kTestHeight);

  EXPECT_FALSE(cylinder.IsNull());

  // Check bounding box
  auto bbox_size = get_bounding_box_size(cylinder);
  EXPECT_NEAR(bbox_size.x(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kTestHeight, test_constants::kCurvedGeometryTolerance);
}

TEST_F(ShapeLoaderTest, MakeCylinder_WithDefaultCenter_IsCenteredAtOrigin)
{
  auto cylinder = loader_->make_cylinder(test_constants::kTestRadius, test_constants::kTestHeight);

  auto center = get_bounding_box_center(cylinder);
  EXPECT_NEAR(center.x(), 0.0, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(center.y(), 0.0, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(center.z(), 0.0, test_constants::kCurvedGeometryTolerance);
}

TEST_F(ShapeLoaderTest, MakeCylinder_WithTranslation_IsCenteredAtSpecifiedPosition)
{
  Eigen::Vector3d center(0.5, 0.5, 0.5);

  auto cylinder = loader_->make_cylinder(test_constants::kTestRadius, test_constants::kTestHeight,
    center);

  auto bbox_center = get_bounding_box_center(cylinder);
  EXPECT_NEAR(bbox_center.x(), 0.5, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_center.y(), 0.5, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_center.z(), 0.5, test_constants::kCurvedGeometryTolerance);
}

TEST_F(ShapeLoaderTest, MakeCylinder_VolumeCalculation_MatchesExpectedVolume)
{
  auto cylinder = loader_->make_cylinder(test_constants::kTestRadius, test_constants::kTestHeight);

  double expected_volume = M_PI * test_constants::kTestRadius * test_constants::kTestRadius *
    test_constants::kTestHeight;
  double actual_volume = get_volume(cylinder);

  EXPECT_NEAR(actual_volume, expected_volume, test_constants::kVolumeTolerance);
}

TEST_F(ShapeLoaderTest, MakeSphere_WithBasicRadius_HasCorrectBoundingBox)
{
  auto sphere = loader_->make_sphere(test_constants::kTestRadius);

  EXPECT_FALSE(sphere.IsNull());

  // Check bounding box (should be cube with side = 2*radius)
  auto bbox_size = get_bounding_box_size(sphere);
  EXPECT_NEAR(bbox_size.x(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
}

TEST_F(ShapeLoaderTest, MakeSphere_WithDefaultCenter_IsCenteredAtOrigin)
{
  auto sphere = loader_->make_sphere(test_constants::kTestRadius);

  auto center = get_bounding_box_center(sphere);
  EXPECT_NEAR(center.x(), 0.0, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(center.y(), 0.0, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(center.z(), 0.0, test_constants::kCurvedGeometryTolerance);
}

TEST_F(ShapeLoaderTest, MakeSphere_WithTranslation_IsCenteredAtSpecifiedPosition)
{
  Eigen::Vector3d center(
    test_constants::kOffsetX,
    test_constants::kOffsetY,
    test_constants::kOffsetZ);

  auto sphere = loader_->make_sphere(test_constants::kTestRadius, center);

  auto bbox_center = get_bounding_box_center(sphere);
  EXPECT_NEAR(bbox_center.x(), test_constants::kOffsetX, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_center.y(), test_constants::kOffsetY, test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_center.z(), test_constants::kOffsetZ, test_constants::kCurvedGeometryTolerance);
}

TEST_F(ShapeLoaderTest, MakeSphere_VolumeCalculation_MatchesExpectedVolume)
{
  auto sphere = loader_->make_sphere(test_constants::kTestRadius);

  double expected_volume = (4.0 / 3.0) * M_PI * test_constants::kTestRadius *
    test_constants::kTestRadius * test_constants::kTestRadius;
  double actual_volume = get_volume(sphere);

  EXPECT_NEAR(actual_volume, expected_volume, test_constants::kVolumeTolerance);
}

TEST_F(ShapeLoaderTest, MakeGroundPlane_WithDefaultParams_HasExpectedDimensions)
{
  auto ground = loader_->make_ground_plane();

  EXPECT_FALSE(ground.IsNull());

  auto bbox_size = get_bounding_box_size(ground);
  EXPECT_NEAR(bbox_size.x(), test_constants::kGroundPlaneSize, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kGroundPlaneSize, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kGroundPlaneThickness,
    test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, MakeGroundPlane_AtZeroHeight_HasTopSurfaceAtZero)
{
  auto ground = loader_->make_ground_plane(
    test_constants::kGroundPlaneSize,
    test_constants::kGroundPlaneSize,
    0.0,
    test_constants::kGroundPlaneThickness);

  // Top surface should be at z=0
  Bnd_Box box;
  BRepBndLib::Add(ground, box);

  double x_min, y_min, z_min, x_max, y_max, z_max;
  box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

  EXPECT_NEAR(z_max, 0.0, test_constants::kPositionTolerance);
  EXPECT_NEAR(z_min, -test_constants::kGroundPlaneThickness, test_constants::kPositionTolerance);
}

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

  // Top surface should be at z=-0.5
  Bnd_Box box;
  BRepBndLib::Add(ground, box);

  double x_min, y_min, z_min, x_max, y_max, z_max;
  box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

  EXPECT_NEAR(z_max, custom_z_position, test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, CombineShapes_WithEmptyVector_Throws)
{
  std::vector<TopoDS_Shape> empty_shapes;
  EXPECT_THROW(loader_->combine_shapes(empty_shapes), std::runtime_error);
}

TEST_F(ShapeLoaderTest, CombineShapes_WithSingleShape_ReturnsSameShape)
{
  auto box = loader_->make_box(Eigen::Vector3d(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));
  std::vector<TopoDS_Shape> shapes = {box};

  auto combined = loader_->combine_shapes(shapes);

  EXPECT_FALSE(combined.IsNull());
  // Single shape should be returned as-is
  EXPECT_EQ(count_faces(combined), count_faces(box));
}

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

  // Combined shape should be a compound
  EXPECT_EQ(combined.ShapeType(), TopAbs_COMPOUND);

  // Total faces should be sum of individual faces
  int box_faces = count_faces(box);
  int sphere_faces = count_faces(sphere);
  int cylinder_faces = count_faces(cylinder);
  int combined_faces = count_faces(combined);

  EXPECT_EQ(combined_faces, box_faces + sphere_faces + cylinder_faces);
}

TEST_F(ShapeLoaderTest, ApplyTransform_WithTranslationOnly_MovesShapeToCorrectPosition)
{
  auto box = loader_->make_box(Eigen::Vector3d(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));
  Eigen::Vector3d translation(
    test_constants::kOffsetX,
    test_constants::kOffsetY,
    test_constants::kOffsetZ);

  auto transformed = loader_->apply_transform(box, translation, Eigen::Quaterniond::Identity());

  auto center = get_bounding_box_center(transformed);
  EXPECT_NEAR(center.x(), test_constants::kOffsetX, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.y(), test_constants::kOffsetY, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.z(), test_constants::kOffsetZ, test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, ApplyTransform_WithRotationOnly_RotatesShapeCorrectly)
{
  // Create box longer in X direction
  auto box = loader_->make_box(Eigen::Vector3d(
    test_constants::kMediumDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));

  // Rotate 90 degrees around Z
  Eigen::Quaterniond rotation(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  auto transformed = loader_->apply_transform(box, Eigen::Vector3d::Zero(), rotation);

  auto bbox_size = get_bounding_box_size(transformed);
  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
}

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

  // Check rotation effect
  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);

  // Check translation
  EXPECT_NEAR(center.x(), test_constants::kOffsetX, test_constants::kPositionTolerance);
  EXPECT_NEAR(center.y(), 0.0, test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, LoadFromStep_WithNonexistentFile_ThrowsRuntimeError)
{
  EXPECT_THROW(
    loader_->load_from_step("/nonexistent/path/file.step"),
    std::runtime_error);
}

TEST_F(ShapeLoaderTest, LoadFromStl_WithNonexistentFile_ThrowsRuntimeError)
{
  EXPECT_THROW(
    loader_->load_from_stl("/nonexistent/path/file.stl"),
    std::runtime_error);
}

TEST_F(ShapeLoaderTest, LoadFromUrdf_WithNonexistentFile_ThrowsRuntimeError)
{
  EXPECT_THROW(
    loader_->load_from_urdf("/nonexistent/path/file.urdf"),
    std::runtime_error);
}

TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithBoxGeometry_CreatesCorrectShape)
{
  std::string urdf_string =
    R"(
    <robot name="test_robot">
      <link name="base_link">
        <collision>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
            <box size="0.1 0.2 0.3"/>
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  auto shape = loader_->load_from_urdf_string(urdf_string);

  EXPECT_FALSE(shape.IsNull());

  auto bbox_size = get_bounding_box_size(shape);
  EXPECT_NEAR(bbox_size.x(), test_constants::kSmallDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension, test_constants::kPositionTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kLargeDimension, test_constants::kPositionTolerance);
}

TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithCylinderGeometry_CreatesCorrectShape)
{
  std::string urdf_string =
    R"(
    <robot name="test_robot">
      <link name="base_link">
        <collision>
          <origin xyz="0 0 0" rpy="0 0 0"/>
          <geometry>
            <cylinder radius="0.05" length="0.1"/>
          </geometry>
        </collision>
      </link>
    </robot>
  )";

  auto shape = loader_->load_from_urdf_string(urdf_string);

  EXPECT_FALSE(shape.IsNull());

  auto bbox_size = get_bounding_box_size(shape);
  EXPECT_NEAR(bbox_size.x(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), 2 * test_constants::kTestRadius,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kTestHeight, test_constants::kCurvedGeometryTolerance);
}

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
  // Sphere radius is 0.1 in this test, so diameter is 0.2
  EXPECT_NEAR(bbox_size.x(), test_constants::kMediumDimension,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.y(), test_constants::kMediumDimension,
    test_constants::kCurvedGeometryTolerance);
  EXPECT_NEAR(bbox_size.z(), test_constants::kMediumDimension,
    test_constants::kCurvedGeometryTolerance);
}

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

  // Should have faces from both box and sphere
  int face_count = count_faces(shape);
  EXPECT_GT(face_count, test_constants::kBoxFaceCount);  // Box has 6, sphere adds more
}

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

  // Should throw because no collision geometry found
  EXPECT_THROW(
    loader_->load_from_urdf_string(urdf_string),
    std::runtime_error);
}

TEST_F(ShapeLoaderTest, LoadFromUrdfString_WithInvalidXml_ThrowsRuntimeError)
{
  std::string urdf_string = "not valid xml <<<<";

  EXPECT_THROW(
    loader_->load_from_urdf_string(urdf_string),
    std::runtime_error);
}

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

TEST_F(ShapeLoaderTest, CreateSecondaryShapesForConstraint_CombinesMultipleShapes)
{
  // Simulate creating secondary shapes for KissingSurfaceConstraint
  auto ground = loader_->make_ground_plane(test_constants::kGroundPlaneSize,
    test_constants::kGroundPlaneSize, 0.0);
  auto fixture = loader_->make_box(
    Eigen::Vector3d(
      test_constants::kSmallDimension,
      test_constants::kSmallDimension,
      test_constants::kMediumDimension),
    Eigen::Vector3d(test_constants::kLargeDimension, 0.0, test_constants::kSmallDimension));

  std::vector<TopoDS_Shape> secondaries = {ground, fixture};

  // All shapes should be valid
  for (const auto & shape : secondaries) {
    EXPECT_FALSE(shape.IsNull());
  }

  // Could combine them if needed
  auto combined = loader_->combine_shapes(secondaries);
  EXPECT_FALSE(combined.IsNull());
}

TEST_F(ShapeLoaderTest, TriangulateExplicitly_WithAutoTriangulateDisabled_SuccessfullyTriangulates)
{
  // Create with auto-triangulate off
  ShapeLoaderConfig config;
  config.auto_triangulate = false;
  ShapeLoader loader(config);

  auto box = loader.make_box(Eigen::Vector3d(
    test_constants::kSmallDimension,
    test_constants::kSmallDimension,
    test_constants::kSmallDimension));
  EXPECT_FALSE(box.IsNull());

  // Manually triangulate
  loader.triangulate(box);

  // Shape should still be valid
  EXPECT_FALSE(box.IsNull());
  EXPECT_EQ(count_faces(box), test_constants::kBoxFaceCount);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
