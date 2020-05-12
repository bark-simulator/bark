// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_RULE_BASED_RULE_BASED_HPP_
#define MODULES_MODELS_BEHAVIOR_RULE_BASED_RULE_BASED_HPP_

#include <memory>
#include <tuple>
#include <utility>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/world/world.hpp"
#include "modules/models/behavior/idm/idm_lane_tracking.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::map::LaneCorridor;
using modules::world::ObservedWorld;
using modules::world::map::LaneCorridorPtr;


enum LaneChangeDecision {
  KeepLane = 0,
  ChangeLeft = 1,
  ChangeRight = 2,
  Undefined = 3
};


enum RuleBasedState {
  Idle = 0,
  IsChanging = 1
};


class BehaviorRuleBased : public BehaviorIDMLaneTracking {
 public:
  explicit BehaviorRuleBased(const commons::ParamsPtr& params) :
    BehaviorIDMLaneTracking(params) {}

  virtual ~BehaviorRuleBased() {}

  /**
   * @brief Returns a trajectory for the ego vehicle
   * 
   * @param delta_time Planning step time (e.g. 0.2s)
   * @param observed_world ObserverdWorld of the ego vehicle
   * @return Trajectory Trajectory at least delta_time long
   */
  virtual Trajectory Plan(
    float delta_time, const ObservedWorld& observed_world);

  /**
   * @brief Function that decides if a lane change is beneficial
   * 
   * @param observed_world ObserverdWorld of the ego vehicle
   * @return std::pair<LaneChangeDecision, LaneCorridorPtr> decision type and
   *                                                        which LaneCorr.
   */
  virtual std::pair<LaneChangeDecision, LaneCorridorPtr>
    CheckIfLaneChangeBeneficial(const ObservedWorld& observed_world) const {}

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorRuleBased::Clone() const {
  std::shared_ptr<BehaviorRuleBased> model_ptr =
      std::make_shared<BehaviorRuleBased>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_RULE_BASED_RULE_BASED_HPP_
