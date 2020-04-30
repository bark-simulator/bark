// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
#define MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_

#include <memory>
#include <Eigen/Dense>


#include "modules/commons/commons.hpp"
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
struct LonLatAction {
  Continuous1DAction acc_lat;
  Continuous1DAction acc_lon;

  inline bool operator==(const LonLatAction& other) {
       return acc_lat == other.acc_lat && acc_lon == other.acc_lon;
    }
};
using dynamic::Input;
typedef boost::variant<DiscreteAction, Continuous1DAction, Input, LonLatAction> Action;
typedef std::size_t ActionHash;

typedef std::pair<models::dynamic::State, Action> StateActionPair;
typedef std::vector<StateActionPair> StateActionHistory;

struct action_tostring_visitor : boost::static_visitor<std::string>
{
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

    std::string operator()(modules::models::behavior::LonLatAction const& val) const {
        std::stringstream ss;
        ss << "LonLatAction: acc_lon=" << val.acc_lat << ", acc_lat=" << val.acc_lat;
        return ss.str();
    }
};


class BehaviorModel : public modules::commons::BaseType {
 public:
  explicit BehaviorModel(const commons::ParamsPtr& params) :
    commons::BaseType(params),
    last_trajectory_(),
    last_action_() {}

  BehaviorModel(const BehaviorModel &behavior_model) :
    commons::BaseType(behavior_model.GetParams()),
    last_trajectory_(behavior_model.GetLastTrajectory()),
    last_action_(behavior_model.GetLastAction()),
    active_model_(behavior_model.GetActiveModel()) {}

  virtual ~BehaviorModel() {}

  dynamic::Trajectory GetLastTrajectory() const { return last_trajectory_; }

  void SetLastTrajectory(const dynamic::Trajectory& trajectory) {
    last_trajectory_ = trajectory;
  }
  bool GetActiveModel() const { return active_model_; }
  virtual Trajectory Plan(float delta_time,
                          const world::ObservedWorld& observed_world) = 0;

  virtual std::shared_ptr<BehaviorModel> Clone() const = 0;

  Action GetLastAction() const {return last_action_; }
  void SetLastAction(const Action action) {last_action_ = action;}

 private:
  dynamic::Trajectory last_trajectory_;
  Action last_action_;
  bool active_model_;
};

typedef std::shared_ptr<BehaviorModel> BehaviorModelPtr;



}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_BEHAVIOR_MODEL_HPP_
