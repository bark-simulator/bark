// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
#define MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_

#include <memory>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorIDMClassic : public BehaviorModel {
 public:
  explicit BehaviorIDMClassic(const commons::ParamsPtr& params);

  virtual ~BehaviorIDMClassic() {}

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world);

  double CalcFreeRoadTerm(const double vel_ego) const;
  double CalcInteractionTerm(const double net_distance, const double vel_ego,
                             const double vel_other) const;

  double CalcNetDistance(
      const std::shared_ptr<const world::objects::Agent>& ego_agent,
      const std::shared_ptr<const world::objects::Agent>& leading_agent);

  double CalcRawIDMAcc(const double& net_distance, const double& vel_ego,
                       const double& vel_other) const;

  double CalcIDMAcc(const double net_distance, const double vel_ego,
                    const double vel_other) const;

  virtual float GetMinVelocity() { return param_min_velocity_; }
  virtual float GetMaxVelocity() { return param_max_velocity_; }
  const double GetDesiredVelocity() const {
    return param_desired_velocity_;
  }  // unit is meter/second
  const float GetMinimumSpacing() const { return param_minimum_spacing_; }      // unit is meter
  const float GetDesiredTimeHeadway() const { return param_desired_time_head_way_; }  // unit is seconds
  const float GetMaxAcceleration() const {
    return param_max_acceleration_;
  }  // unit is meter/second^2
  const float GetAccelerationLowerBound() const {
    return param_acceleration_lower_bound_;
  } 
  const float GetAccelerationUpperBound() const {
    return param_acceleration_upper_bound_;
  } 
  const float GetComfortableBrakingAcceleration() const {
    return param_comfortable_braking_acceleration_;
  }  // unit is meter/second^2
  const int GetExponent() const { return param_exponent_; }

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  private:
    // Parameters
    float param_minimum_spacing_;
    float param_desired_time_head_way_;
    float param_max_acceleration_;
    float param_acceleration_lower_bound_;
    float param_acceleration_upper_bound_;
    float param_desired_velocity_;
    float param_comfortable_braking_acceleration_;
    float param_min_velocity_;
    float param_max_velocity_;
    int param_exponent_;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMClassic::Clone() const {
  std::shared_ptr<BehaviorIDMClassic> model_ptr =
      std::make_shared<BehaviorIDMClassic>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
