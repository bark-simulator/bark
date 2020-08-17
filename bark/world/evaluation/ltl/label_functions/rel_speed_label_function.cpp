// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/label_functions/rel_speed_label_function.hpp"

#include "bark/world/observed_world.hpp"

using bark::models::dynamic::StateDefinition;

bark::world::evaluation::RelSpeedLabelFunction::RelSpeedLabelFunction(
    const std::string& string, const double rel_speed_thres)
    : MultiAgentLabelFunction(string), rel_speed_thres_(rel_speed_thres) {}

bool bark::world::evaluation::RelSpeedLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
    const AgentPtr& other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  if (other_agent) {
    const float rel_speed =
        ego_agent->GetCurrentState()(StateDefinition::VEL_POSITION) -
        other_agent->GetCurrentState()(StateDefinition::VEL_POSITION);
    return rel_speed >= rel_speed_thres_;
  }
  return false;
}
