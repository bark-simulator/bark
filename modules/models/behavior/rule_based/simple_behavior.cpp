// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/models/behavior/rule_based/simple_behavior.hpp"
#include "modules/models/behavior/idm/base_idm.hpp"
#include <algorithm>
#include <memory>
#include <utility>
#include <limits>
#include <tuple>
#include "modules/models/dynamic/integration.hpp"
#include "modules/models/dynamic/single_track.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::State;
using dynamic::StateDefinition;
using modules::commons::transformation::FrenetPosition;
using modules::geometry::Line;
using modules::geometry::Point2d;
using modules::models::dynamic::CalculateSteeringAngle;
using modules::models::dynamic::DynamicModelPtr;
using StateDefinition::VEL_POSITION;
using world::Agent;
using world::AgentFrenetPair;
using world::AgentId;
using world::ObservedWorld;
using world::map::LaneCorridorPtr;
using world::map::RoadCorridorPtr;
using world::objects::AgentPtr;

std::pair<LaneChangeDecision, LaneCorridorPtr>
BehaviorSimpleRuleBased::CheckIfLaneChangeBeneficial(
  const ObservedWorld& observed_world) const {
  auto lane_corr = observed_world.GetLaneCorridor();
  LaneChangeDecision change_decision = LaneChangeDecision::KeepLane;

  // if enough space on ego corr do not change
  std::tuple<double, double, bool> relative_values =
    BaseIDM::CalcRelativeValues(
      observed_world, lane_corr);

  // if there is not enough space
  if (std::get<0>(relative_values) < 20.) {
    const auto& road_corr = observed_world.GetRoadCorridor();

    if (fabs(std::get<1>(relative_values) / observed_world.CurrentEgoState()[StateDefinition::VEL_POSITION]) < 0.1) {
      // std::cout << std::get<0>(relative_values) << std::endl;
      std::pair<LaneCorridorPtr, LaneCorridorPtr> left_right_lane_corr =
        road_corr->GetLeftRightLaneCorridor(
          observed_world.CurrentEgoPosition());

      // we want to change lanes if there is more free space
      if (left_right_lane_corr.first) {
        std::tuple<double, double, bool> relative_values_left_lane =
          BaseIDM::CalcRelativeValues(
            observed_world, left_right_lane_corr.first);
        if (std::get<0>(relative_values_left_lane) >= std::get<0>(relative_values))
          return std::pair<LaneChangeDecision, LaneCorridorPtr>(
            LaneChangeDecision::ChangeLeft, left_right_lane_corr.first);
      }
    }

  }



  return std::pair<LaneChangeDecision, LaneCorridorPtr>(
    change_decision, lane_corr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
