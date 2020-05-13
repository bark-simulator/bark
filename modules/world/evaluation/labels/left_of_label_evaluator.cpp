// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "left_of_label_evaluator.hpp"
#include "modules/world/observed_world.hpp"
bool modules::world::evaluation::LeftOfLabelEvaluator::evaluate_agent(
    const modules::world::ObservedWorld& observed_world,
    const modules::world::objects::AgentPtr& other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  const auto ego_lc = observed_world.GetLaneCorridor();
  if (other_agent) {
    const auto other_lc =
        other_agent->GetRoadCorridor()
            ->GetLeftRightLaneCorridor(other_agent->GetCurrentPosition())
            .first;
    return other_lc == ego_lc;
  }
  return false;
}
