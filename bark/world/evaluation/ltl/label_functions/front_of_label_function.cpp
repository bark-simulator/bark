// Copyright (c) 2019 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "front_of_label_function.hpp"

#include "bark/world/observed_world.hpp"

using bark::commons::transformation::FrenetPosition;

bool bark::world::evaluation::FrontOfLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
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
      return ((f_other.lon + other_agent->GetShape().front_dist_) <=
              (f_ego.lon - ego_agent->GetShape().rear_dist_));
    }
  }
  return false;
}
