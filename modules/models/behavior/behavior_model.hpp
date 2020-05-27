// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "modules/commons/base_type.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace world {
namespace objects {
class Agent;
typedef std::shared_ptr<Agent> AgentPtr;
typedef unsigned int AgentId;
}  // namespace objects
class ObservedWorld;
}  // namespace world
namespace models {
namespace behavior {
using dynamic::Trajectory;

typedef unsigned int DiscreteAction;
typedef double Continuous1DAction;
using dynamic::Input;
typedef boost::variant<DiscreteAction, Continuous1DAction, Input> Action;

typedef std::pair<models::dynamic::State, Action> StateActionPair;
typedef std::vector<StateActionPair> StateActionHistory;

enum BehaviorStatus : unsigned int {
  NOT_STARTED_YET = 0,
  VALID = 1,
  EXPIRED = 2
};

class BehaviorModel : public modules::commons::BaseType {
 public:
  explicit BehaviorModel(const commons::ParamsPtr& params,
                         BehaviorStatus status)
      : commons::BaseType(params),
        last_trajectory_(),
        last_action_(),
        behavior_status_(status) {}

  explicit BehaviorModel(const commons::ParamsPtr& params)
      : BehaviorModel(params, BehaviorStatus::VALID) {}

  BehaviorModel(const BehaviorModel& behavior_model)
      : commons::BaseType(behavior_model.GetParams()),
        last_trajectory_(behavior_model.GetLastTrajectory()),
        last_action_(behavior_model.GetLastAction()),
        behavior_status_(behavior_model.GetBehaviorStatus()) {}

  virtual ~BehaviorModel() {}

  dynamic::Trajectory GetLastTrajectory() const { return last_trajectory_; }

  void SetLastTrajectory(const dynamic::Trajectory& trajectory) {
    last_trajectory_ = trajectory;
  }

  BehaviorStatus GetBehaviorStatus() const { return behavior_status_; }

  void SetBehaviorStatus(const BehaviorStatus status) {
    behavior_status_ = status;
  }

  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world) = 0;

  virtual void ActionToBehavior(const Action& action) {};

  virtual std::shared_ptr<BehaviorModel> Clone() const {};

  Action GetLastAction() const { return last_action_; }
  void SetLastAction(const Action action) { last_action_ = action; }

 private:
  dynamic::Trajectory last_trajectory_;
  Action last_action_;
  BehaviorStatus behavior_status_;
};

typedef std::shared_ptr<BehaviorModel> BehaviorModelPtr;

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
