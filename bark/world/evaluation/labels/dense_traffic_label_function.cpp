// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/labels/dense_traffic_label_function.hpp"

#include <algorithm>
#include "bark/world/observed_world.hpp"

namespace modules {
namespace world {
namespace evaluation {

using modules::models::dynamic::StateDefinition;

DenseTrafficLabelFunction::DenseTrafficLabelFunction(
    const std::string& label_str, double radius, size_t num_agents)
    : BaseLabelFunction(label_str),
      radius_(std::abs(radius)),
      num_agents_(num_agents) {}
std::vector<LabelMap::value_type> DenseTrafficLabelFunction::Evaluate(
    const world::ObservedWorld& observed_world) const {
  size_t agent_count = 0;
  const auto& ego_pos = observed_world.GetEgoAgent()->GetCurrentPosition();
  // TODO(@cirrostratus1): Use rtree for performance
  for (const auto& agent : observed_world.GetOtherAgents()) {
    const auto& other_pos = agent.second->GetCurrentPosition();
    if (std::abs(geometry::Distance(ego_pos, other_pos)) < radius_) {
      ++agent_count;
    }
  }
  return {{GetLabel(), agent_count >= num_agents_}};
}
}  // namespace evaluation
}  // namespace world
}  // namespace modules
