// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_BEHAVIOR_RSS_HPP_
#define BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_BEHAVIOR_RSS_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#ifdef RSS
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#endif
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;
using world::evaluation::BaseEvaluator;
using world::objects::AgentId;
using bark::models::behavior::BehaviorIDMClassic;
using bark::models::behavior::BehaviorSafety;
using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;
#ifdef RSS
using bark::world::evaluation::EvaluatorRSS;
#endif
enum class BehaviorRSSConformantStatus {SAFETY_BEHAVIOR, NOMINAL_BEHAVIOR};

class BehaviorRSSConformant : public BehaviorModel {
 public:
  explicit BehaviorRSSConformant(const commons::ParamsPtr& params) :
    BehaviorModel(params),
    nominal_behavior_model_(std::make_shared<BehaviorIDMLaneTracking>(params)),
    behavior_safety_model_(std::make_shared<BehaviorSafety>(params)),
    rss_evaluator_(),
    behavior_rss_status_(BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR),
    world_time_of_last_rss_violation_(-1),
    initial_lane_corr_(nullptr) {
      try {
        #ifdef RSS
        rss_evaluator_ = std::make_shared<EvaluatorRSS>(params);
        #endif
      } catch (...) {
        VLOG(4) << "Could not load RSSEvaluator." << std::endl;
      }
    }

  virtual ~BehaviorRSSConformant() {}

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  // setter and getter
  std::shared_ptr<BehaviorModel> GetNominalBehaviorModel() const {
    return nominal_behavior_model_;
  }
  void SetNominalBehaviorModel(const std::shared_ptr<BehaviorModel>& model){
    nominal_behavior_model_ = model;
  } 
  std::shared_ptr<BehaviorSafety> GetBehaviorSafetyModel() const {
    return behavior_safety_model_;
  }
  void SetSafetyBehaviorModel(const std::shared_ptr<BehaviorSafety>& model){
    behavior_safety_model_ = model;
  }

  void SetEvaluator(const std::shared_ptr<BaseEvaluator>& evaluator){
    rss_evaluator_ = evaluator;
  }

  int32_t GetLongitudinalResponse() const { return lon_; }
  int32_t GetLateralLeftResponse() const { return lat_left_; }
  int32_t GetLateralRightResponse() const { return lat_right_; }
  std::vector<uint64_t> GetDangerousObjectIdsResponse() const {
    return dangerous_objects_;
  }

  void SetLongitudinalResponse(int32_t lon) { lon_ = lon; }
  void SetLateralLeftResponse(int32_t lat_left) { lat_left_ = lat_left; }
  void SetLateralRightResponse(int32_t lat_right) { lat_right_ = lat_right; }
  void SetDangerousObjectIdsResponse(
    const std::vector<uint64_t>& ids) { dangerous_objects_ = ids; }

 private:
  std::shared_ptr<BehaviorModel> nominal_behavior_model_;
  std::shared_ptr<BehaviorSafety> behavior_safety_model_;
  std::shared_ptr<BaseEvaluator> rss_evaluator_;
  BehaviorRSSConformantStatus behavior_rss_status_;
  float world_time_of_last_rss_violation_;
  LaneCorridorPtr initial_lane_corr_;

  int32_t lon_{0}, lat_left_{0}, lat_right_{0};
  std::vector<uint64_t> dangerous_objects_{};
};

inline std::shared_ptr<BehaviorModel> BehaviorRSSConformant::Clone() const {
  std::shared_ptr<BehaviorRSSConformant> model_ptr =
      std::make_shared<BehaviorRSSConformant>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_RSS_BEHAVIOR_BEHAVIOR_RSS_HPP_
