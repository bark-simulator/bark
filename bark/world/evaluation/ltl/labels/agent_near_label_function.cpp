// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/labels/agent_near_label_function.hpp"

#include "bark/world/observed_world.hpp"

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Distance;

modules::world::evaluation::AgentNearLabelFunction::AgentNearLabelFunction(
    const std::string &string, const double distance_thres)
    : MultiAgentLabelFunction(string), distance_thres_(distance_thres) {
  assert(distance_thres_ >= 0.0);
}

bool modules::world::evaluation::AgentNearLabelFunction::EvaluateAgent(
    const modules::world::ObservedWorld &observed_world,
    const AgentPtr &other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  if (other_agent) {
    const auto &poly_ego =
        ego_agent->GetPolygonFromState(ego_agent->GetCurrentState());
    const auto &poly_other =
        other_agent->GetPolygonFromState(other_agent->GetCurrentState());
    return std::abs(Distance(poly_ego, poly_other)) < distance_thres_;
  }
  return false;
}
