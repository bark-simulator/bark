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
#include "bark/models/behavior/behavior_safety/behavior_safety.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#ifdef RSS
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#endif
#include "bark/world/evaluation/rss/safety_polygon.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::models::behavior::BehaviorIDMClassic;
using bark::models::behavior::BehaviorSafety;
using bark::models::dynamic::AccelerationLimits;
using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::evaluation::BaseEvaluator;
using world::evaluation::ComputeSafetyPolygon;
using world::evaluation::SafetyPolygon;
using world::objects::AgentId;
#ifdef RSS
using bark::world::evaluation::EvaluatorRSS;
#endif
enum class BehaviorRSSConformantStatus { SAFETY_BEHAVIOR, NOMINAL_BEHAVIOR };

template <typename Enumeration>
auto as_integer(Enumeration const value) {
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

class BehaviorRSSConformant : public BehaviorModel {
 public:
  explicit BehaviorRSSConformant(const commons::ParamsPtr& params)
      : BehaviorModel(params),
        nominal_behavior_model_(std::make_shared<BehaviorIDMLaneTracking>(
            GetParams()->AddChild("NominalBehavior"))),
        behavior_safety_model_(std::make_shared<BehaviorSafety>(GetParams())),
        rss_evaluator_(),
        behavior_rss_status_(BehaviorRSSConformantStatus::NOMINAL_BEHAVIOR),
        world_time_of_last_rss_violation_(-1),
        initial_lane_corr_(nullptr),
        minimum_safety_corridor_length_(GetParams()->GetReal(
            "MinimumSafetyCorridorLength",
            "Minimal lenght a safety corridor should have that a lateral "
            "safety maneuver is performed.",
            0.f)),
        acceleration_limits_(),
        acc_restrictions_for_nominal_(GetParams()->GetBool(
            "AccRestrictionsForNominal",
            "Restrict Nominal Model using Acc Limits", false)),
        acc_restrictions_for_safety_(GetParams()->GetBool(
            "AccRestrictionsForSafety",
            "Restrict Safety Model using Acc Limits", false)),
        no_safety_maneuver_(GetParams()->GetBool(
            "NoSafetyManeuver", "No triggering of safety maneuver", false)) {
    try {
#ifdef RSS
      rss_evaluator_ = std::make_shared<EvaluatorRSS>(GetParams());
#endif
    } catch (...) {
      VLOG(4) << "Could not load RSSEvaluator." << std::endl;
    }
  }

  virtual ~BehaviorRSSConformant() {}

  Trajectory Plan(double min_planning_time,
                  const ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  // setter and getter
  std::shared_ptr<BehaviorModel> GetNominalBehaviorModel() const {
    return nominal_behavior_model_;
  }

  BehaviorRSSConformantStatus GetBehaviorRssStatus() const {
    return behavior_rss_status_;
  }

  void SetNominalBehaviorModel(const std::shared_ptr<BehaviorModel>& model) {
    nominal_behavior_model_ = model;
  }
  std::shared_ptr<BehaviorSafety> GetBehaviorSafetyModel() const {
    return behavior_safety_model_;
  }
  void SetSafetyBehaviorModel(const std::shared_ptr<BehaviorSafety>& model) {
    behavior_safety_model_ = model;
  }

  void SetEvaluator(const std::shared_ptr<BaseEvaluator>& evaluator) {
    rss_evaluator_ = evaluator;
  }

  void SetAccelerationLimits(const AccelerationLimits& limits) {
    acceleration_limits_ = limits;
  }

  AccelerationLimits GetAccelerationLimits() const {
    return acceleration_limits_;
  }

#ifdef RSS
  AccelerationLimits ConvertRestrictions(
      const ::ad::rss::state::AccelerationRestriction& acc_restrictions);

  void ApplyRestrictionsToModel(const AccelerationLimits& limits,
                                std::shared_ptr<BehaviorModel> model);

  int32_t GetLongitudinalResponse() const { return as_integer(lon_response_); }
  int32_t GetLateralLeftResponse() const {
    return as_integer(lat_left_response_);
  }
  int32_t GetLateralRightResponse() const {
    return as_integer(lat_right_response_);
  }
  void SetLongitudinalResponse(int32_t lon) {
    lon_response_ = static_cast<::ad::rss::state::LongitudinalResponse>(lon);
  }
  void SetLateralLeftResponse(int32_t lat_left) {
    lat_left_response_ =
        static_cast<::ad::rss::state::LateralResponse>(lat_left);
  }
  void SetLateralRightResponse(int32_t lat_right) {
    lat_right_response_ =
        static_cast<::ad::rss::state::LateralResponse>(lat_right);
  }
  void ComputeSafetyPolygons(const ObservedWorld& observed_world) {
    for (auto& sp : safety_polygons_) ComputeSafetyPolygon(sp, observed_world);
  }

#endif
  std::vector<SafetyPolygon> GetSafetyPolygons() const {
    return safety_polygons_;
  }
  void SetSafetyPolygons(const std::vector<SafetyPolygon>& sp) {
    safety_polygons_ = sp;
  }

 private:
  std::shared_ptr<BehaviorModel> nominal_behavior_model_;
  std::shared_ptr<BehaviorSafety> behavior_safety_model_;
  std::shared_ptr<BaseEvaluator> rss_evaluator_;
  BehaviorRSSConformantStatus behavior_rss_status_;
  double world_time_of_last_rss_violation_;
  LaneCorridorPtr initial_lane_corr_;
  float minimum_safety_corridor_length_;
  AccelerationLimits acceleration_limits_;
  bool acc_restrictions_for_nominal_;
  bool acc_restrictions_for_safety_;
  bool no_safety_maneuver_;
#ifdef RSS
  ::ad::rss::state::LongitudinalResponse lon_response_;
  ::ad::rss::state::LateralResponse lat_left_response_;
  ::ad::rss::state::LateralResponse lat_right_response_;
  ::ad::rss::state::AccelerationRestriction acc_restrictions_;
#endif
  std::vector<SafetyPolygon> safety_polygons_;
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
