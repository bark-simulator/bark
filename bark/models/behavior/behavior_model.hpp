// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
#define BARK_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "bark/commons/commons.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
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
struct LonLatAction {
  Continuous1DAction acc_lat;
  Continuous1DAction acc_lon;

  inline bool operator==(const LonLatAction& other) const {
    return acc_lat == other.acc_lat && acc_lon == other.acc_lon;
  }
};
using dynamic::Input;
using models::dynamic::State;
typedef boost::variant<DiscreteAction, Continuous1DAction, Input, LonLatAction>
    Action;
typedef std::size_t ActionHash;

typedef std::pair<State, Action> StateActionPair;
typedef std::vector<StateActionPair> StateActionHistory;

typedef std::vector<State> StateHistory;
typedef std::vector<Action> ActionHistory;

struct action_tostring_visitor : boost::static_visitor<std::string> {
  std::string operator()(DiscreteAction const& val) const {
    std::stringstream ss;
    ss << "Discrete Action: " << val;
    return ss.str();
  }

  std::string operator()(Continuous1DAction const& val) const {
    std::stringstream ss;
    ss << "Continuous1DAction: " << val;
    return ss.str();
  }

  std::string operator()(Input const& val) const {
    std::stringstream ss;
    ss << "ActionInput: " << val;
    return ss.str();
  }

  std::string operator()(
      bark::models::behavior::LonLatAction const& val) const {
    std::stringstream ss;
    ss << "LonLatAction: acc_lon=" << val.acc_lat
       << ", acc_lat=" << val.acc_lat;
    return ss.str();
  }
};

enum BehaviorStatus : unsigned int {
  NOT_STARTED_YET = 0,
  VALID = 1,
  EXPIRED = 2
};

class BehaviorModel : public bark::commons::BaseType {
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

  virtual Trajectory Plan(float min_planning_time,
                          const world::ObservedWorld& observed_world) = 0;

  virtual std::shared_ptr<BehaviorModel> Clone() const = 0;

  Action GetLastAction() const { return last_action_; }
  void SetLastAction(const Action& action) { last_action_ = action; }

  // externally set action that can also be set from Python
  Action GetAction() const { return action_to_behavior_; }
  virtual void ActionToBehavior(const Action& action) {
    action_to_behavior_ = action;
  };

 private:
  dynamic::Trajectory last_trajectory_;
  // can either be the last action or action to be executed
  Action last_action_;
  Action action_to_behavior_;
  BehaviorStatus behavior_status_;
};

typedef std::shared_ptr<BehaviorModel> BehaviorModelPtr;

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
