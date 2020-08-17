// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "right_of_label_function.hpp"

#include "bark/world/observed_world.hpp"

using bark::commons::transformation::FrenetPosition;

bool bark::world::evaluation::RightOfLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
    const AgentPtr& other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  const auto ego_lc = observed_world.GetLaneCorridor();
  if (other_agent) {
    const auto other_lc =
        other_agent->GetRoadCorridor()
            ->GetLeftRightLaneCorridor(other_agent->GetCurrentPosition())
            .second;
    return other_lc && ego_lc && *other_lc == *ego_lc;
  }
  return false;
}
