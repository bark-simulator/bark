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

  // TODO(@hart): check all LaneCorridors in the RoadCorridor
  const LaneCorridorPtr& ego_lane_corr = observed_world.GetLaneCorridor();

  // distances
  FrenetPosition frenet_ego(
    observed_world.CurrentEgoPosition(), ego_lane_corr->GetCenterLine());

  const RoadCorridorPtr& ego_road_corr = observed_world.GetRoadCorridor();
  // TODO(@hart): only use neighboring LaneCorridors
  for (const auto& lane_corr : ego_road_corr->GetUniqueLaneCorridors()) {
    const auto& front_rear_agent =  observed_world.GetAgentFrontRear(lane_corr);
    const auto& vehicle_front = front_rear_agent.front;
    const auto& vehicle_behind = front_rear_agent.rear;
    double distance_front = std::numeric_limits<double>::max();
    double distance_behind = -std::numeric_limits<double>::max();
    // get distances projected on ego lane_corr
    if (vehicle_front.first) {
      FrenetPosition frenet_vehicle_front(
        vehicle_front.first->GetCurrentPosition(),
        ego_lane_corr->GetCenterLine());
      distance_front = frenet_vehicle_front.lon;
    }
    if (vehicle_behind.first) {
      FrenetPosition frenet_vehicle_behind(
        vehicle_behind.first->GetCurrentPosition(),
        ego_lane_corr->GetCenterLine());
        distance_behind = frenet_vehicle_behind.lon;
    }

    // relative distances
    // TODO(@hart): also include velocities!
    distance_front = distance_front - frenet_ego.lon;
    distance_behind = distance_behind - frenet_ego.lon;
  }

  // we start with a negative offset behind the vehicle
  // then we calc. free space and put as idx in map
  // sort map by keys
  // if we are close to another object and there is free space
  // on the other lanes --> change lanes

  auto lane_corr = observed_world.GetLaneCorridor();
  LaneChangeDecision change_decision = LaneChangeDecision::KeepLane;

  return std::pair<LaneChangeDecision, LaneCorridorPtr>(
    change_decision, lane_corr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules
