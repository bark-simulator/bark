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

enum class BehaviorRSSConformantStatus {SAFE_BEHAVIOR, NOMINAL_BEHAVIOR};

class BehaviorRSSConformant : public BehaviorModel {
 public:
  explicit BehaviorRSSConformant(const commons::ParamsPtr& params) :
    BehaviorModel(params),
    rss_behavior_status_(BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR),
    world_time_of_last_rss_violation_(-1) {}

  virtual ~BehaviorRSSConformant() {}

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  void SetNominalBehaviorModel(const std::shared_ptr<BehaviorModel>& model){
    nominal_behavior_model_ = model;
  } 

  void SetSafetyBehaviorModel(const std::shared_ptr<BehaviorModel>& model){
    safety_behavior_model_ = model;
  }

 private:
  std::shared_ptr<BehaviorModel> nominal_behavior_model_;
  std::shared_ptr<BehaviorModel> safety_behavior_model_;
  // TODO: needs to be the RSS evaluator
  std::shared_ptr<BaseEvaluator> rss_evaluator_;
  BehaviorRSSConformantStatus rss_behavior_status_;
  float world_time_of_last_rss_violation_;
};

inline std::shared_ptr<BehaviorModel> BehaviorRSSConformant::Clone() const {
  std::shared_ptr<BehaviorRSSConformant> model_ptr =
      std::make_shared<BehaviorRSSConformant>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_RSS_BEHAVIOR_HPP_
