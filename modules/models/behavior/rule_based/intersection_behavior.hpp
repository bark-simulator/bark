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

#include "modules/models/behavior/rule_based/lane_change_behavior.hpp"
#include "modules/models/behavior/idm/base_idm.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::Agent;
using modules::world::AgentId;
using modules::world::AgentPtr;
using modules::world::FrenetPosition;
using modules::world::map::LaneCorridorPtr;
using modules::world::ObservedWorld;
using modules::world::AgentMap;
using modules::world::prediction::PredictionSettings;
using modules::world::AgentFrenetPair;
using modules::models::dynamic::StateDefinition::VEL_POSITION;


// Behavior for intersection; prediction-based; right before left
class BehaviorIntersectionRuleBased : public BehaviorLaneChangeRuleBased {
 public:
  explicit BehaviorIntersectionRuleBased(
    const commons::ParamsPtr& params) :
    BehaviorLaneChangeRuleBased(params) {
    // this is required for the IDM to get around corners
    SetLimitSteeringRate(false);
    // parameters
    prediction_time_horizon_ = params->GetReal(
      "BehaviorIntersectionRuleBased::PredictionTimeHorizon",
      "Prediction time horizon.",
      5.0);
    prediction_t_inc_ = params->GetReal(
      "BehaviorIntersectionRuleBased::PredictionTInc",
      "Fine graining of prediction collision checking.",
      0.5);
    braking_distance_ = params->GetReal(
      "BehaviorIntersectionRuleBased::BrakingDistance",
      "Distance at which the vehicle should start to brake.",
      10.);
    angle_diff_for_intersection_ = params->GetReal(
      "BehaviorIntersectionRuleBased::AngleDiffForIntersection",
      "Angle at which vehicles are counted as intersecting.",
      1.4);
  }

  Trajectory Plan(
    float delta_time, const world::ObservedWorld& observed_world);

  std::tuple<double, AgentPtr> CheckIntersectingVehicles(
    const ObservedWorld& observed_world,
    double t_inc = 0.5);

  std::pair<AgentId, bool> GetIntersectingAgent(
    const AgentMap& intersecting_agents,
    const ObservedWorld& observed_world) const;

  virtual ~BehaviorIntersectionRuleBased() {}

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  double prediction_time_horizon_;
  double prediction_t_inc_;
  double angle_diff_for_intersection_;
  double braking_distance_;
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


