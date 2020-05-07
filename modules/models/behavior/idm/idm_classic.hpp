// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
#define MODULES_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_

#include <memory>
#include <tuple>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/models/behavior/longitudinal_acceleration/longitudinal_acceleration.hpp"
#include "modules/world/world.hpp"
#include "base_idm.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::map::LaneCorridor;
using modules::world::map::LaneCorridorPtr;


class BehaviorIDMClassic : public BaseIDM {
 public:
  explicit BehaviorIDMClassic(const commons::ParamsPtr& params) :
    BaseIDM(params) {}

  virtual ~BehaviorIDMClassic() {}

  Trajectory Plan(float delta_time, const ObservedWorld& observed_world);

  std::tuple<Trajectory, Action> GenerateTrajectory(
    const world::ObservedWorld& observed_world,
    const std::tuple<double, double, bool>& rel_values,
    float delta_time) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;
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
