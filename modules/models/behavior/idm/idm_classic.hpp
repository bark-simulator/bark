// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
#define MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_

#include "modules/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "modules/world/world.hpp"

namespace modules {
namespace models {
namespace behavior {

class BehaviorIDMClassic : public BehaviorLongitudinalAcceleration {
 public:
  explicit BehaviorIDMClassic(commons::Params *params) :
    BehaviorLongitudinalAcceleration(params) {}

  virtual ~BehaviorIDMClassic() {}

  virtual double CalculateLongitudinalAcceleration(const world::ObservedWorld& observed_world);

  std::pair<world::objects::AgentPtr, modules::world::map::Frenet> GetLeadingVehicle(const world::ObservedWorld& observed_world);

  const double get_desired_velocity() {return 15.0f;} // // unit is meter/second
  const float get_minimum_spacing() {return 2.0f;} // unit is meter
  const float get_desired_time_headway() {return 1.5f;} // unit is seconds
  const float get_max_acceleration() {return 1.7f;} // unit is meter/second^2
  const float get_comfortable_braking_acceleration() {return 1.67f;} // unit is meter/second^2
  const int get_exponent() { return  4;}

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMClassic::Clone() const {
  std::shared_ptr<BehaviorIDMClassic> model_ptr =
    std::make_shared<BehaviorIDMClassic>(*this);
  return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
