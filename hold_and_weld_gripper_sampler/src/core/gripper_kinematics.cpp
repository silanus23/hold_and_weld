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
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Compound.hxx>

namespace hold_and_weld_gripper_sampler
{
namespace core
{

TopoDS_Shape configure_gripper(const ParsedGripper & gripper, double grip_distance)
{
  if (gripper.finger_1.IsNull() || gripper.finger_2.IsNull() || gripper.base.IsNull()) {
    throw std::runtime_error("configure_gripper: Input shapes are null");
  }

  const double finger_travel = std::max(0.0,
    std::min(grip_distance / 2.0, gripper.max_opening / 2.0));

  try {
    gp_Trsf f1_trsf;
    f1_trsf.SetTranslation(gp_Vec(gripper.finger_1_axis.x(),
                                  gripper.finger_1_axis.y(),
                                  gripper.finger_1_axis.z()) * finger_travel);

    gp_Trsf f2_trsf;
    f2_trsf.SetTranslation(gp_Vec(gripper.finger_2_axis.x(),
                                  gripper.finger_2_axis.y(),
                                  gripper.finger_2_axis.z()) * finger_travel);

    BRepBuilderAPI_Transform f1_transformer(gripper.finger_1, f1_trsf, Standard_True);
    if (!f1_transformer.IsDone()) {
      throw std::runtime_error("configure_gripper: BRepBuilderAPI_Transform failed for finger_1");
    }
    BRepBuilderAPI_Transform f2_transformer(gripper.finger_2, f2_trsf, Standard_True);
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
    builder.Add(compound, gripper.base);

    return compound;
  } catch (const Standard_Failure & e) {
    throw std::runtime_error(std::string("OCCT Transform failure: ") + e.GetMessageString());
  }
}

}  // namespace core
}  // namespace hold_and_weld_gripper_sampler
