// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_
#define MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_

#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

enum LaneChangeDecision { KeepLane = 0, ChangeLeft = 1, ChangeRight = 2 };

// From article "MOBIL: General Lane-Changing Model for Car-Following Models"
class BehaviorMobil : public BehaviorIDMClassic {
 public:
  explicit BehaviorMobil(commons::Params *params)
      : BehaviorIDMClassic(params),
        is_changing_lane_(false),
        has_changed_lane_(false) {
    politeness_ = params->GetReal(
        "PolitenessFactor", "Politness factor, suggested [0.2, 0.5]", 0.35f);

    //! Acceleration bias needs to be larger than the acceleration threshold
    acceleration_threshold_ = params->GetReal(
        "AccelerationThreshold",
        "Models intertia to only trigger if there is real improvement", 0.1f);

    acceleration_bias_ = params->GetReal(
        "AccelerationBias", "Bias to encourage keep right directive", 0.1f);

    safe_deceleration_ = params->GetReal(
        "SafeDeceleration",
        "Maximum deceleration for follower in target lane, positive number",
        2.0f);

    asymmetric_passing_rules_ = params->GetBool(
        "AsymmetricPassingRules",
        "Whether passing on the right side is forbidden", false);

    critical_velocity_ = params->GetReal(
        "CriticalVelocity",
        "Passing on the right side is allowed below this velocity",
        16.66f);  // 16.66 m/s = 60 km/h
  }

  virtual ~BehaviorMobil() {}

  Trajectory Plan(float delta_time, const world::ObservedWorld &observed_world);

  LaneChangeDecision CheckIfLaneChangeBeneficial(
      const world::ObservedWorld &observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  bool is_changing_lane_;
  bool has_changed_lane_;

  // Measure of altruism, defines how the model weighs an acceleration
  // improvement for the ego agent against an improvement for other agents
  float politeness_;

  // Models intertia to only trigger if there is real improvement
  float acceleration_threshold_;

  // Bias to encourage keep right directive
  float acceleration_bias_;

  // Maximum deceleration for follower in target lane, positive number
  float safe_deceleration_;

  // Whether passing on the right side is forbidden
  bool asymmetric_passing_rules_;

  // Passing on the right side is allowed below this velocity
  float critical_velocity_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMobil::Clone() const {
  std::shared_ptr<BehaviorMobil> model_ptr =
      std::make_shared<BehaviorMobil>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_