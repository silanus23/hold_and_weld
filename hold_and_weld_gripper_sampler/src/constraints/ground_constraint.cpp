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

#include "hold_and_weld_gripper_sampler/constraints/ground_constraint.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <rclcpp/rclcpp.hpp>
#include <TopoDS_Wire.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace constraints
{

GroundConstraint::GroundConstraint(const GroundConfig & config)
: config_(config),
  logger_(rclcpp::get_logger("gripper_sampler"))
{
  RCLCPP_DEBUG(logger_,
    "GroundConstraint: z=%.4f m, footprint %.2f x %.2f m centred on (%.3f, %.3f), "
    "band=%.4f m, support_threshold=%.1f%%",
    config_.bottom_z, config_.size_x, config_.size_y,
    config_.center_x, config_.center_y,
    config_.contact_band, config_.support_threshold * 100.0);
}

void GroundConstraint::analyze_constraints(const geometry::Topology & topology)
{
  banned_surface_ids_.clear();
  partial_exclusions_.clear();

  const auto & all_surfaces = topology.get_all_surfaces();

  RCLCPP_INFO(logger_,
    "Analyzing ground contact: %zu surface(s), z=%.4f m, footprint %.2f x %.2f m",
    all_surfaces.size(), config_.bottom_z, config_.size_x, config_.size_y);

  for (size_t i = 0; i < all_surfaces.size(); ++i) {
    const int surface_id = static_cast<int>(i);
    const TopoDS_Face & face = all_surfaces[i].face;

    std::vector<sampling::FaceSample> resting_samples;
    const double support = measure_ground_support(face, &resting_samples);

    if (support < 1e-9) {
      continue;
    }

    if (support > config_.support_threshold) {
      banned_surface_ids_.push_back(surface_id);
      RCLCPP_DEBUG(logger_, "Surface %d: %.1f%% resting -> banned", surface_id, support * 100.0);
      continue;
    }

    const TopoDS_Wire boundary = sampling::bounding_wire_in_uv(face, resting_samples);
    if (boundary.IsNull()) {
      RCLCPP_WARN(logger_, "Surface %d: %.1f%% resting but wire extraction failed — "
        "ground contact not excluded from sampling on this face",
        surface_id, support * 100.0);
      continue;
    }

    core::SampleArea area;
    area.surface_id = surface_id;
    area.wire = boundary;
    area.is_exclusion = true;
    partial_exclusions_.push_back(area);
    RCLCPP_DEBUG(logger_, "Surface %d: %.1f%% resting -> exclusion wire",
      surface_id, support * 100.0);
  }

  RCLCPP_INFO(logger_, "Ground analysis complete: %zu banned, %zu partial exclusions",
    banned_surface_ids_.size(), partial_exclusions_.size());
}

double GroundConstraint::measure_ground_support(
  const TopoDS_Face & face,
  std::vector<sampling::FaceSample> * resting_samples) const
{
  if (resting_samples != nullptr) {
    resting_samples->clear();
  }

  // Cheap reject, if lowest point is above don't bother checking.
  Bnd_Box face_box;
  BRepBndLib::Add(face, face_box);
  if (face_box.IsVoid()) {
    return 0.0;
  }

  Standard_Real x_min, y_min, z_min, x_max, y_max, z_max;
  face_box.Get(x_min, y_min, z_min, x_max, y_max, z_max);

  const double ceiling = config_.bottom_z + config_.contact_band;
  if (z_min > ceiling) {
    return 0.0;
  }
  const double foot_x_min = config_.center_x - config_.size_x * 0.5;
  const double foot_x_max = config_.center_x + config_.size_x * 0.5;
  const double foot_y_min = config_.center_y - config_.size_y * 0.5;
  const double foot_y_max = config_.center_y + config_.size_y * 0.5;
  if (x_max < foot_x_min || x_min > foot_x_max ||
    y_max < foot_y_min || y_min > foot_y_max)
  {
    return 0.0;
  }

  sampling::FaceSamplingConfig sampling_config;
  sampling_config.sample_density = config_.sample_density;
  sampling_config.layout = sampling::GridLayout::kCellCentres;

  const auto samples = sampling::sample_face_region(face, sampling_config);
  if (samples.empty()) {
    RCLCPP_DEBUG(logger_, "Face near the ground footprint produced no samples "
      "(sample_density=%.4f m too coarse for this face?) — treating as 0%% resting",
      config_.sample_density);
    return 0.0;
  }

  // Predicate does double duty: it's the resting test area_fraction counts against,
  // and (when the caller wants them) it collects the passing samples in the same
  // pass instead of re-walking `samples` afterward.
  return sampling::area_fraction(
    samples,
    [this, ceiling, resting_samples](const sampling::FaceSample & sample) {
      if (sample.point.Z() > ceiling) {return false;}
      if (!is_within_footprint(sample.point.X(), sample.point.Y())) {return false;}
      if (resting_samples != nullptr) {
        resting_samples->push_back(sample);
      }
      return true;
    });
}

bool GroundConstraint::intersects_ground(
  const gp_Trsf & gripper_transform, double grip_distance) const
{
  if (!fcl_checker_ || !fcl_checker_->is_valid()) {
    RCLCPP_ERROR(logger_, "FCL checker not available — rejecting grasp conservatively");
    return true;
  }
  return fcl_checker_->collides_with_ground(
    gripper_transform, grip_distance, config_.collision_tolerance);
}

bool GroundConstraint::is_within_footprint(double x, double y) const
{
  const double half_x = config_.size_x * 0.5;
  const double half_y = config_.size_y * 0.5;
  return std::abs(x - config_.center_x) <= half_x &&
         std::abs(y - config_.center_y) <= half_y;
}

std::vector<int> GroundConstraint::get_banned_surface_ids() const
{
  return banned_surface_ids_;
}

std::vector<core::SampleArea> GroundConstraint::get_sample_areas() const
{
  return partial_exclusions_;
}

void GroundConstraint::set_fcl_checker(
  std::shared_ptr<const geometry::FCLCollisionChecker> fcl_checker)
{
  fcl_checker_ = fcl_checker;
}

std::string GroundConstraint::get_name() const
{
  return "GroundConstraint";
}

}  // namespace constraints
}  // namespace hold_and_weld_gripper_sampler
