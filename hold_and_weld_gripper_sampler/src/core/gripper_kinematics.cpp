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

#include "hold_and_weld_gripper_sampler/core/gripper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Compound.hxx>

namespace hold_and_weld_gripper_sampler
{

TopoDS_Shape ParsedGripper::configure(double grip_distance) const
{
  if (finger_1.IsNull() || finger_2.IsNull() || base.IsNull()) {
    throw std::runtime_error("configure_gripper: Input shapes are null");
  }

  const double finger_travel = std::max(0.0,
    std::min(grip_distance / 2.0, max_opening / 2.0));

  // Normalise: a non-unit axis would silently scale finger_travel.
  auto safe_unit_vec = [](const Eigen::Vector3d & ax, const char * name) -> gp_Vec {
      const double mag = ax.norm();
      if (mag < 1e-9) {
        throw std::runtime_error(
        std::string("configure_gripper: ") + name + " has zero magnitude");
      }
      if (std::abs(mag - 1.0) > 1e-3) {
        RCLCPP_WARN(
        rclcpp::get_logger("gripper_sampler"),
        "configure_gripper: %s has non-unit magnitude %.6f, normalising", name, mag);
      }
      return gp_Vec(ax.x() / mag, ax.y() / mag, ax.z() / mag);
    };

  try {
    gp_Trsf f1_trsf;
    f1_trsf.SetTranslation(
      safe_unit_vec(finger_1_axis, "finger_1_axis") * finger_travel);

    gp_Trsf f2_trsf;
    f2_trsf.SetTranslation(
      safe_unit_vec(finger_2_axis, "finger_2_axis") * finger_travel);

    BRepBuilderAPI_Transform f1_transformer(finger_1, f1_trsf, Standard_True);
    if (!f1_transformer.IsDone()) {
      throw std::runtime_error("configure_gripper: BRepBuilderAPI_Transform failed for finger_1");
    }
    BRepBuilderAPI_Transform f2_transformer(finger_2, f2_trsf, Standard_True);
    if (!f2_transformer.IsDone()) {
      throw std::runtime_error("configure_gripper: BRepBuilderAPI_Transform failed for finger_2");
    }
    TopoDS_Shape f1_opened = f1_transformer.Shape();
    TopoDS_Shape f2_opened = f2_transformer.Shape();

    if (f1_opened.IsNull() || f2_opened.IsNull()) {
      throw std::runtime_error("configure_gripper: Transformation yielded null shape");
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    builder.Add(compound, f1_opened);
    builder.Add(compound, f2_opened);
    builder.Add(compound, base);

    return compound;
  } catch (const Standard_Failure & e) {
    throw std::runtime_error(std::string("OCCT Transform failure: ") + e.GetMessageString());
  }
}

}  // namespace hold_and_weld_gripper_sampler
