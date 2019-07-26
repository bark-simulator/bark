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

using dynamic::Trajectory;
using world::objects::AgentId;
using world::ObservedWorld;

class BehaviorIDMClassic : public BehaviorLongitudinalAcceleration {
 public:
  explicit BehaviorIDMClassic(commons::Params *params) :
    BehaviorLongitudinalAcceleration(params) {}

  virtual ~BehaviorIDMClassic() {}

  virtual double CalculateLongitudinalAcceleration(const ObservedWorld& observed_world);

  modules::world::objects::AgentPtr GetLeadingVehicle(const ObservedWorld& observed_world);

  virtual BehaviorModel *Clone() const;
};

inline BehaviorModel *BehaviorIDMClassic::Clone() const {
  return new BehaviorIDMClassic(*this);
}

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
