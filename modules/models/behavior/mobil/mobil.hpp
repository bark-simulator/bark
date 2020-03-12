// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_
#define MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_

#include <memory>
#include <utility>

#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/world/observed_world.hpp"

namespace modules {
namespace models {
namespace behavior {

enum LaneChangeDecision { KeepLane = 0, ChangeLeft = 1, ChangeRight = 2 };
enum MobilState { Idle = 0, IsChanging = 1 };

// From article "MOBIL: General Lane-Changing Model for Car-Following Models"
class BehaviorMobil : public BehaviorModel {
 public:
  explicit BehaviorMobil(const commons::ParamsPtr& params)
      : BehaviorModel(params),
    idm_(std::make_shared<BehaviorIDMClassic>(params->AddChild("BehaviorMobil"))),
    mobil_state_(MobilState::Idle) {
    crosstrack_error_gain_ = params->GetReal("BehaviorMobil::CrosstrackErrorGain", "Tuning factor of stanley controller", 1.0);
    politeness_ = params->GetReal("BehaviorMobil::PolitenessFactor", "Politness factor, suggested [0.2, 0.5]", 0.35f);

    //! Acceleration bias needs to be larger than the acceleration threshold
    acceleration_threshold_ = params->GetReal("BehaviorMobil::AccelerationThreshold", "Models intertia to only trigger if there is real improvement", 0.1f);
    acceleration_bias_ = params->GetReal("BehaviorMobil::AccelerationBias", "Bias to encourage keep right directive", 0.1f);
    safe_deceleration_ = params->GetReal("BehaviorMobil::SafeDeceleration", "Maximum deceleration for follower in target lane, positive number", 2.0f);
    asymmetric_passing_rules_ = params->GetBool("BehaviorMobil::AsymmetricPassingRules", "Whether passing on the right side is forbidden", false);
    critical_velocity_ = params->GetReal("BehaviorMobil::CriticalVelocity", "Passing on the right side is allowed below this velocity", 16.66f);  // 16.66 m/s = 60 km/h
    stop_at_lane_ending_ = params->GetBool("BehaviorMobil::StopAtLaneEnding", "Flag to let vehicle slow down at lane ending", true);
  }

  virtual ~BehaviorMobil() {}

  Trajectory Plan(float delta_time, const world::ObservedWorld& observed_world);

  double CalcLongRawAccWithoutLeader(
      const world::LaneCorridorPtr& lane_corridor,
      const modules::geometry::Point2d& pos, const float vel);

  double CalcNetDistanceFromFrenet(
      const std::shared_ptr<const world::objects::Agent>& ego_agent,
      const modules::commons::transformation::FrenetPosition& ego_frenet,
      const std::shared_ptr<const world::objects::Agent>& leading_agent,
      const modules::commons::transformation::FrenetPosition& leading_frenet)
      const;

  std::pair<LaneChangeDecision, world::map::LaneCorridorPtr>
  CheckIfLaneChangeBeneficial(const world::ObservedWorld& observed_world);

  virtual std::shared_ptr<BehaviorModel> Clone() const;

 private:
  std::shared_ptr<BehaviorIDMClassic> idm_;

  MobilState mobil_state_;

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
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_
