#ifndef MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_
#define MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_

#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "modules/models/behavior/pure_pursuit/pure_pursuit.hpp"
#include "modules/world/observed_world.hpp"


namespace modules {
namespace models {
namespace behavior {

// From article "MOBIL: General Lane-Changing Model for Car-Following Models"
class BehaviorMobil : public BehaviorModel {
 public:
  explicit BehaviorMobil(const dynamic::DynamicModelPtr &dynamic_model,
                         const BehaviorModelPtr &longitudinal_behavior,
                         const float politeness,
                         const float acceleration_threshold,
                         commons::Params *params)
      : BehaviorModel(params),
        dynamic_model_(dynamic_model),
        behavior_pure_pursuit_(params),
        is_changing_lane_(false),
        politeness_(politeness),
        acceleration_threshold_(acceleration_threshold) {
    longitudinal_behavior_ =
        std::dynamic_pointer_cast<BehaviorLongitudinalAcceleration>(
            longitudinal_behavior);
    if (!longitudinal_behavior) {
      std::cerr << "The parameter longitudinal_behavior of BehaviorMobil"
                << "must be a subclass of BehaviorLongitudinalAcceleration."
                << std::endl;
      throw std::bad_cast();
    }

    safe_decel_ = params->get_real(
        "SafeDeceleration",
        "The maximum deceleration that is considered safe, a positive number",
        2.0f);

    asymmetric_passing_rules_ = params->get_bool(
        "AsymmetricPassingRules",
        "Whether passing on the right side is disallowed",
        false);

    critical_velocity_ = params->get_real(
        "CriticalVelocity",
        "Passing on the right side is always allowed below this velocity",
        16.66f);  // 16.66 m/s = 60 km/h

    acceleration_bias_ = params->get_real(
      "AccelerationBias",
      "Bias to encourage staying on the rightmost lane",
      0.1f);

    // Acceleration bias needs to be larger than the acceleration threshold
    acceleration_bias_ += acceleration_threshold_;
  }

  ~BehaviorMobil() {}

  Trajectory Plan(float delta_time, const world::ObservedWorld &observed_world);

  void InitiateLaneChangeIfBeneficial(
    const world::ObservedWorld &observed_world);

  std::shared_ptr<BehaviorModel> Clone() const;
  
 private:
  double CalculateLongitudinalAcceleration(
    const std::shared_ptr<const world::objects::Agent> &ego_agent,
    const std::shared_ptr<const world::objects::Agent> &leading_vehicle,
    const double distance) const;

  dynamic::DynamicModelPtr dynamic_model_;

  std::shared_ptr<BehaviorLongitudinalAcceleration> longitudinal_behavior_;
  BehaviorPurePursuit behavior_pure_pursuit_;

  bool is_changing_lane_;

  // Measure of altruism, defines how the model weighs an acceleration
  // improvement for the ego agent against an improvement for other agents
  float politeness_;

  // Minimum acceleration improvement that triggers a lane change
  float acceleration_threshold_;

  // Maximum deceleration that is considered safe, a positive number
  float safe_decel_;

  // True if passing is only allowed on the left
  bool asymmetric_passing_rules_;

  // Passing on the right is allowed below this velocity
  // (i.e. in congested traffic)
  float critical_velocity_;

  // Bias to encourage staying on the rightmost lane
  float acceleration_bias_;
};

inline std::shared_ptr<BehaviorModel> BehaviorMobil::Clone() const {
  return std::make_shared<BehaviorMobil>(*this);
}

} // namespace behavior
} // namespace models
} // namespace modules

#endif // MODULES_MODELS_BEHAVIOR_MOBIL_MOBIL_HPP_