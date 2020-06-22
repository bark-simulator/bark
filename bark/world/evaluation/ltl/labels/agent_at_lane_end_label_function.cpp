// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/labels/agent_at_lane_end_label_function.hpp"

#include "bark/world/observed_world.hpp"

using modules::commons::transformation::FrenetPosition;
using modules::geometry::Distance;

modules::world::evaluation::AgentAtLaneEndLabelFunction::
    AgentAtLaneEndLabelFunction(const std::string &string,
                                const double distance_thres)
    : MultiAgentLabelFunction(string), distance_thres_(distance_thres) {
  assert(distance_thres_ >= 0.0);
}

bool modules::world::evaluation::AgentAtLaneEndLabelFunction::EvaluateAgent(
    const modules::world::ObservedWorld &observed_world,
    const AgentPtr &other_agent) const {
  if (other_agent) {
    const auto &other_pos = other_agent->GetCurrentPosition();
    const auto &lc =
        other_agent->GetRoadCorridor()->GetNearestLaneCorridor(other_pos);
    const float dist_until_end =
        lc->LengthUntilEnd(other_pos) - other_agent->GetShape().front_dist_;
    return std::abs(dist_until_end) < distance_thres_;
  }
  return false;
}
