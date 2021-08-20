// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_NOT_STARTED_BEHAVIOR_NOT_STARTED_HPP_
#define BARK_MODELS_BEHAVIOR_NOT_STARTED_BEHAVIOR_NOT_STARTED_HPP_

#include <memory>
#include <utility>
#include <vector>

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace behavior {

using dynamic::Trajectory;
using world::ObservedWorld;

// model that always has behavior status not started (to be used in search,
// replacing BehaviorStaticTrajectory, to prevent that BehaviorStaticTrajectory
// becomes true in forward search and thus suddenly unexpectingly shows up)
class BehaviorNotStarted : public BehaviorModel {
 public:
  explicit BehaviorNotStarted(const commons::ParamsPtr& params)
      : BehaviorModel(params, BehaviorStatus::NOT_STARTED_YET) {
    SetLastAction(LonLatAction{0.0, 0.0});
  }

  Trajectory Plan(double min_planning_time,
                  const world::ObservedWorld& observed_world) {
    UpdateBehaviorStatus(min_planning_time, observed_world);

    auto traj = dynamic::Trajectory();
    this->SetLastAction(LonLatAction{0.0, 0.0});
    this->SetLastTrajectory(traj);
    return traj;
  };

  std::shared_ptr<BehaviorModel> Clone() const {
    std::shared_ptr<BehaviorNotStarted> model_ptr =
        std::make_shared<BehaviorNotStarted>(*this);
    return std::dynamic_pointer_cast<BehaviorModel>(model_ptr);
  };

  void UpdateBehaviorStatus(double delta_time,
                            const world::ObservedWorld& observed_world) {
    SetBehaviorStatus(BehaviorStatus::NOT_STARTED_YET);
  }
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_NOT_STARTED_BEHAVIOR_NOT_STARTED_HPP_
