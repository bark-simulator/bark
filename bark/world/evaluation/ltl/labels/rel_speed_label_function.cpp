// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/labels/rel_speed_label_function.hpp"

#include "bark/world/observed_world.hpp"

using modules::models::dynamic::StateDefinition;

modules::world::evaluation::RelSpeedLabelFunction::RelSpeedLabelFunction(
    const std::string &string, const double rel_speed_thres)
    : MultiAgentLabelFunction(string), rel_speed_thres_(rel_speed_thres) {}

bool modules::world::evaluation::RelSpeedLabelFunction::EvaluateAgent(
    const modules::world::ObservedWorld &observed_world,
    const AgentPtr &other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  if (other_agent) {
    const float rel_speed =
        ego_agent->GetCurrentState()(StateDefinition::VEL_POSITION) -
        other_agent->GetCurrentState()(StateDefinition::VEL_POSITION);
    return rel_speed >= rel_speed_thres_;
  }
  return false;
}
