// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_STATIC_TRAJECTORY_BEHAVIOR_STATIC_TRAJECTORY_HPP_
#define MODULES_MODELS_BEHAVIOR_STATIC_TRAJECTORY_BEHAVIOR_STATIC_TRAJECTORY_HPP_

#include <memory>
#include <vector>
#include <utility>

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace behavior {

using dynamic::DynamicModelPtr;
using dynamic::Input;
using dynamic::State;
using dynamic::Trajectory;
using world::ObservedWorld;
using world::objects::AgentId;
using StateRowVector = Eigen::Matrix<State::Scalar, 1, Eigen::Dynamic>;


// model for replaying static trajectories
// can e.g. be used for dataset replay
class BehaviorStaticTrajectory : public BehaviorModel {
 public:
  explicit BehaviorStaticTrajectory(const commons::ParamsPtr& params);
  BehaviorStaticTrajectory(const commons::ParamsPtr& params,
                           const Trajectory& static_trajectory);
  Trajectory Plan(float delta_time,
                  const world::ObservedWorld& observed_world) override;
  std::shared_ptr<BehaviorModel> Clone() const override;
  const Trajectory& get_static_trajectory() const;
  void UpdateBehaviorStatus(float delta_time,
                            const world::ObservedWorld& observed_world);

 private:
  static Trajectory ReadInStaticTrajectory(
    std::vector<std::vector<float>> list);
  std::pair<int, int> Interpolate(const double t,
                                  StateRowVector* interpolated) const;
  Trajectory static_trajectory_;
};

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_STATIC_TRAJECTORY_BEHAVIOR_STATIC_TRAJECTORY_HPP_
