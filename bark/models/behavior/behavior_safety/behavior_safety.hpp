// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_SAFETY_BEHAVIOR_BEHAVIOR_SAFETY_HPP_
#define BARK_MODELS_BEHAVIOR_SAFETY_BEHAVIOR_BEHAVIOR_SAFETY_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;
using bark::models::behavior::BehaviorIDMLaneTracking;
using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;
using bark::commons::ParamsPtr;

class BehaviorSafety : public BehaviorModel {
 public:
  explicit BehaviorSafety(const ParamsPtr& params) :
    BehaviorModel(params),
    initial_lane_corr_(nullptr) {
      behavior_safety_params_ = params->AddChild("BehaviorSafety");
      behavior_safety_params_->SetReal("BehaviorIDMClassic::DesiredVelocity", 1.);
      // behavior_safety_params_->SetReal(
      //   "BehaviorIDMClassic::AccelerationLowerBound", -0.5);
      behavior_model_ = std::make_shared<BehaviorIDMLaneTracking>(
        behavior_safety_params_);
  }

  explicit BehaviorSafety(const std::shared_ptr<BehaviorSafety>& bs) :
    BehaviorModel(bs->GetBehaviorModel()->GetParams()),
    behavior_model_(bs->GetBehaviorModel()),
    initial_lane_corr_(bs->GetInitialLaneCorridor()),
    behavior_safety_params_(bs->GetBehaviorSafetyParams()) {}

  virtual ~BehaviorSafety() {}

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SetBehaviorModel(const std::shared_ptr<BehaviorModel>& model){
    behavior_model_ = model;
  } 

  std::shared_ptr<BehaviorModel> GetBehaviorModel() const {
    return behavior_model_;
  }

  void SetInitialLaneCorridor(const LaneCorridorPtr& lc){
    initial_lane_corr_ = lc;
  }

  LaneCorridorPtr GetInitialLaneCorridor() const {
    return initial_lane_corr_;
  }

  ParamsPtr GetBehaviorSafetyParams() const {
    return behavior_safety_params_;
  }

 private:
  std::shared_ptr<BehaviorModel> behavior_model_;
  LaneCorridorPtr initial_lane_corr_;
  ParamsPtr behavior_safety_params_;
};

inline std::shared_ptr<BehaviorModel> BehaviorSafety::Clone() const {
  std::shared_ptr<BehaviorSafety> new_bs = std::make_shared<BehaviorSafety>(*this);
  return new_bs;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_SAFETY_BEHAVIOR_BEHAVIOR_SAFETY_HPP_
