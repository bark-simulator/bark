// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_RSS_BEHAVIOR_HPP_
#define BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_RSS_BEHAVIOR_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;
using world::evaluation::BaseEvaluator;
using world::objects::AgentId;

enum class RSSBehaviorStatus {SAFE_BEHAVIOR, NORMAL_BEHAVIOR};

class RSSBehavior : public BehaviorModel {
 public:
  explicit RSSBehavior(const commons::ParamsPtr& params) :
    BehaviorModel(params),
    rss_behavior_status_(RSSBehaviorStatus::NORMAL_BEHAVIOR) {}

  virtual ~RSSBehavior() {}

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SetBehaviorModel(const std::shared_ptr<BehaviorModel>& model){
    behavior_model_ = model;
  } 

  void SetSafetyBehaviorModel(const std::shared_ptr<BehaviorModel>& model){
    safety_behavior_model_ = model;
  }

 private:
  std::shared_ptr<BehaviorModel> behavior_model_;
  std::shared_ptr<BehaviorModel> safety_behavior_model_;
  // TODO: needs to be the RSS evaluator
  std::shared_ptr<BaseEvaluator> rss_evaluator_;
  RSSBehaviorStatus rss_behavior_status_;
};

inline std::shared_ptr<BehaviorModel> RSSBehavior::Clone() const {
  std::shared_ptr<RSSBehavior> model_ptr =
      std::make_shared<RSSBehavior>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_RSS_BEHAVIOR_HPP_
