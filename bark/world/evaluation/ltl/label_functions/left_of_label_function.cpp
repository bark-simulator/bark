// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "left_of_label_function.hpp"
#include "bark/world/observed_world.hpp"
bool bark::world::evaluation::LeftOfLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
    const bark::world::objects::AgentPtr& other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  const auto ego_lc = observed_world.GetLaneCorridor();
  if (other_agent) {
    const auto other_lc =
        other_agent->GetRoadCorridor()
            ->GetLeftRightLaneCorridor(other_agent->GetCurrentPosition())
            .first;
    return other_lc && ego_lc && *other_lc == *ego_lc;
  }
  return false;
}
