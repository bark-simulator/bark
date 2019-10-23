// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_

#include <memory>
#include <Eigen/Dense>

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


class BehaviorModel : public modules::commons::BaseType {
 public:
  explicit BehaviorModel(commons::Params *params) : commons::BaseType(params),
                                                    last_trajectory_(),
                                                    last_action_() {}

  BehaviorModel(const BehaviorModel &behavior_model) :
    commons::BaseType(behavior_model.get_params()),
    last_trajectory_(behavior_model.get_last_trajectory()),
    last_action_(behavior_model.get_last_action()),
    active_model_(behavior_model.get_active_model()) {}

  virtual ~BehaviorModel() {}

  dynamic::Trajectory get_last_trajectory() const { return last_trajectory_; }

  void set_last_trajectory(const dynamic::Trajectory& trajectory) {
    last_trajectory_ = trajectory;
  }
  bool get_active_model() const { return active_model_; }
  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world) {}

  virtual std::shared_ptr<BehaviorModel> Clone() const;

  Action get_last_action() const {return last_action_; }
  void set_last_action(const Action action) {last_action_ = action;}

 private:
  dynamic::Trajectory last_trajectory_;
  Action last_action_;
  bool active_model_;
};

inline std::shared_ptr<BehaviorModel> BehaviorModel::Clone() const {
  return std::make_shared<BehaviorModel>(*this);
}

typedef std::shared_ptr<BehaviorModel> BehaviorModelPtr;

}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
