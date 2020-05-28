// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "front_of_label_function.hpp"

#include "modules/world/observed_world.hpp"

using modules::commons::transformation::FrenetPosition;

bool modules::world::evaluation::FrontOfLabelFunction::evaluate_agent(
    const modules::world::ObservedWorld& observed_world,
    const AgentPtr& other_agent) const {
  const auto ego_agent = observed_world.GetEgoAgent();
  if (other_agent) {
    const auto other_lane =
        other_agent->GetRoadCorridor()->GetCurrentLaneCorridor(
            other_agent->GetCurrentPosition());
    if (!other_lane) {
      return false;
    } else {
      FrenetPosition f_ego(ego_agent->GetCurrentPosition(),
                           other_lane->GetCenterLine());
      FrenetPosition f_other(other_agent->GetCurrentPosition(),
                             other_lane->GetCenterLine());
      return (f_other.lon <= f_ego.lon);
    }
  }
  return false;
}