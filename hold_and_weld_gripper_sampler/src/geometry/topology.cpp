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

#include "hold_and_weld_gripper_sampler/geometry/topology.hpp"

#include <algorithm>
#include <stdexcept>

namespace hold_and_weld_gripper_sampler
{
namespace geometry
{

const Corner & Topology::get_corner(int id) const
{
  if (id < 0 || id >= static_cast<int>(corners_.size())) {
    throw std::out_of_range("Corner ID " + std::to_string(id) + " not found");
  }
  return corners_[id];
}

const Edge & Topology::get_edge(int id) const
{
  if (id < 0 || id >= static_cast<int>(edges_.size())) {
    throw std::out_of_range("Edge ID " + std::to_string(id) + " not found");
  }
  return edges_[id];
}

const Surface & Topology::get_surface(int id) const
{
  if (id < 0 || id >= static_cast<int>(surfaces_.size())) {
    throw std::out_of_range("Surface ID " + std::to_string(id) + " not found");
  }
  return surfaces_[id];
}

std::vector<int> Topology::get_all_surface_ids() const
{
  std::vector<int> ids;
  ids.reserve(surfaces_.size());

  for (size_t i = 0; i < surfaces_.size(); i++) {
    ids.push_back(static_cast<int>(i));
  }

  return ids;
}

const std::vector<Corner> & Topology::get_all_corners() const
{
  return corners_;
}

const std::vector<Edge> & Topology::get_all_edges() const
{
  return edges_;
}

const std::vector<Surface> & Topology::get_all_surfaces() const
{
  return surfaces_;
}

int Topology::add_corner(const Corner & corner)
{
  int id = static_cast<int>(corners_.size());
  corners_.push_back(corner);
  return id;
}

int Topology::add_edge(const Edge & edge)
{
  int id = static_cast<int>(edges_.size());
  edges_.push_back(edge);
  return id;
}

int Topology::add_surface(const Surface & surface)
{
  int id = static_cast<int>(surfaces_.size());
  surfaces_.push_back(surface);
  return id;
}

void Topology::clear()
{
  corners_.clear();
  edges_.clear();
  surfaces_.clear();
}

}  // namespace geometry
}  // namespace hold_and_weld_gripper_sampler
