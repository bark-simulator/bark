// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "succeeding_agent_label_function.hpp"
#include "bark/world/observed_world.hpp"

using bark::world::AgentFrenetPair;
using bark::world::AgentId;

bark::world::evaluation::SucceedingAgentLabelFunction::
    SucceedingAgentLabelFunction(const std::string& string,
                                bool use_frac_param_from_world,
                                double lateral_difference_threshold)
    : MultiAgentLabelFunction(string),
      use_frac_param_from_world_(use_frac_param_from_world),
      lateral_difference_threshold_(lateral_difference_threshold) {}

bool bark::world::evaluation::SucceedingAgentLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
    const bark::world::objects::AgentPtr& other_agent) const {
  bool is_in_front = false;

  const auto& lane_corridor = observed_world.GetLaneCorridor();
  if (lane_corridor) {
    AgentId id = observed_world.GetEgoAgentId();
    double frac;
    if (use_frac_param_from_world_) {
      frac = observed_world.GetLateralDifferenceThreshold();
    } else {
      frac = lateral_difference_threshold_;
    }
    auto front_rear_agents =
        observed_world.GetAgentFrontRearForId(id, lane_corridor, frac);
    AgentFrenetPair front_agent = front_rear_agents.front;

    if (front_agent.first) {
      is_in_front =
          (front_agent.first->GetAgentId() == other_agent->GetAgentId());
    }
  }
  return is_in_front;
}
