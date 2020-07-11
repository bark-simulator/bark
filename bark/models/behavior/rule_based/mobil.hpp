// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_HPP_
#define BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_HPP_

#include <memory>
#include <utility>

#include "bark/models/behavior/idm/base_idm.hpp"
#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
#include "bark/world/observed_world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::Agent;
using bark::world::FrenetPosition;

// From article "MOBIL: General Lane-Changing Model for Car-Following Models"
class BehaviorMobil : public BehaviorLaneChangeRuleBased {
 public:
  explicit BehaviorMobil(const commons::ParamsPtr& params)
      : BehaviorModel(params), BehaviorLaneChangeRuleBased(params) {
    crosstrack_error_gain_ =
        params->GetReal("BehaviorMobil::CrosstrackErrorGain",
                        "Tuning factor of stanley controller", 1.0);
    politeness_ =
        params->GetReal("BehaviorMobil::PolitenessFactor",
                        "Politness factor, suggested [0.2, 0.5]", 0.35f);
    //! Acceleration bias needs to be larger than the acceleration threshold
    acceleration_threshold_ = params->GetReal(
        "BehaviorMobil::AccelerationThreshold",
        "Models intertia to only trigger if there is real improvement", 0.1f);
    acceleration_bias_ =
        params->GetReal("BehaviorMobil::AccelerationBias",
                        "Bias to encourage keep right directive", 0.1f);
    safe_deceleration_ = params->GetReal(
        "BehaviorMobil::SafeDeceleration",
        "Maximum deceleration for follower in target lane, positive number",
        2.0f);
    asymmetric_passing_rules_ = params->GetBool(
        "BehaviorMobil::AsymmetricPassingRules",
        "Whether passing on the right side is forbidden", false);
    critical_velocity_ = params->GetReal(
        "BehaviorMobil::CriticalVelocity",
        "Passing on the right side is allowed below this velocity",
        16.66f);  // 16.66 m/s = 60 km/h
    stop_at_lane_ending_ =
        params->GetBool("BehaviorMobil::StopAtLaneEnding",
                        "Flag to let vehicle slow down at lane ending", true);
  }

  virtual ~BehaviorMobil() {}

  double CalcNetDistanceFromFrenet(
      const std::shared_ptr<const Agent>& ego_agent,
      const FrenetPosition& ego_frenet,
      const std::shared_ptr<const Agent>& leading_agent,
      const FrenetPosition& leading_frenet) const;

  double CalcLongRawAccWithoutLeader(
      const world::LaneCorridorPtr& lane_corridor,
      const bark::geometry::Point2d& pos, const float vel) const;

  std::pair<LaneChangeDecision, world::map::LaneCorridorPtr>
  CheckIfLaneChangeBeneficial(const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  // std::shared_ptr<BehaviorIDMClassic> idm_;
  // MobilState mobil_state_;
  double crosstrack_error_gain_;
  world::map::LaneCorridorPtr target_corridor_;
  // Measure of altruism, defines how the model weighs an acceleration
  // improvement for the ego agent against an improvement for other agents
  double politeness_;
  // Models intertia to only trigger if there is real improvement
  double acceleration_threshold_;
  // Bias to encourage keep right directive
  double acceleration_bias_;
  // Maximum deceleration for follower in target lane, positive number
  double safe_deceleration_;
  // Whether passing on the right side is forbidden
  bool asymmetric_passing_rules_;
  // Passing on the right side is allowed below this velocity
  double critical_velocity_;
  // Flag to let vehicle slow down (and eventually stop) at lane ending
  bool stop_at_lane_ending_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMobil::Clone() const {
  std::shared_ptr<BehaviorMobil> model_ptr =
      std::make_shared<BehaviorMobil>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_RULE_BASED_MOBIL_HPP_
