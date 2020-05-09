// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_RULE_BASED_INTERSECTION_BEHAVIOR_HPP_
#define MODULES_MODELS_BEHAVIOR_RULE_BASED_INTERSECTION_BEHAVIOR_HPP_

#include <memory>
#include <map>
#include <utility>
#include <vector>
#include <tuple>

#include "modules/models/behavior/rule_based/rule_based.hpp"
#include "modules/models/behavior/rule_based/simple_behavior.hpp"
#include "modules/models/behavior/idm/base_idm.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::Agent;
using modules::world::AgentPtr;
using modules::world::FrenetPosition;
using modules::world::map::LaneCorridorPtr;
using modules::world::ObservedWorld;
using modules::world::AgentMap;
using modules::world::prediction::PredictionSettings;
using modules::world::AgentFrenetPair;
using modules::models::dynamic::StateDefinition::VEL_POSITION;


// this model can change lanes as well as to slow down for other vehicles
// at intersections
class BehaviorIntersectionRuleBased : public BehaviorSimpleRuleBased {
 public:
  explicit BehaviorIntersectionRuleBased(
    const commons::ParamsPtr& params) :
    BehaviorSimpleRuleBased(params) {
    // this is required for the IDM to get around corners
    SetLimitSteeringRate(false);
  }

  Trajectory Plan(
    float delta_time, const world::ObservedWorld& observed_world);

  std::tuple<double, AgentPtr> CheckIntersectingVehicles(
    const LaneCorridorPtr& lane_corr,
    const ObservedWorld& observed_world,
    double pred_horizon = 5.,
    double t_inc = 0.5);

  AgentPtr FilterLaneCorridorIntersectingAgents(
    const AgentMap& intersecting_agents,
    const ObservedWorld& observed_world) const;

  virtual ~BehaviorIntersectionRuleBased() {}

  virtual std::shared_ptr<BehaviorModel> Clone() const;

};

inline std::shared_ptr<BehaviorModel>
BehaviorIntersectionRuleBased::Clone() const {
  std::shared_ptr<BehaviorIntersectionRuleBased> model_ptr =
      std::make_shared<BehaviorIntersectionRuleBased>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_RULE_BASED_INTERSECTION_BEHAVIOR_HPP_


