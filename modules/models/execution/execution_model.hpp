// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_EXECUTION_EXECUTION_MODEL_HPP_
#define MODULES_MODELS_EXECUTION_EXECUTION_MODEL_HPP_

#include <Eigen/Core>
#include <memory>

#include "modules/commons/base_type.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace execution {

using dynamic::DynamicModelPtr;
using dynamic::Trajectory;
using dynamic::State;


enum ExecutionStatus : unsigned int {
  VALID = 0,
  INVALID = 1
};


class ExecutionModel : public commons::BaseType {
 public:
  explicit ExecutionModel(modules::commons::ParamsPtr params) :
    BaseType(params),
    last_trajectory_(),
    execution_status_(ExecutionStatus::VALID) {}

  ExecutionModel(const ExecutionModel& execution_model) :
    BaseType(execution_model.GetParams()),
    last_trajectory_(execution_model.GetLastTrajectory()),
    execution_status_(execution_model.GetExecutionStatus()) {}

  virtual ~ExecutionModel() {}

  Trajectory GetLastTrajectory() const { return last_trajectory_; }

  void SetLastTrajectory(const Trajectory& trajectory) {
    last_trajectory_ = trajectory;
  }

  ExecutionStatus GetExecutionStatus() const {
    return execution_status_;
  }

  virtual Trajectory Execute(const float& new_world_time,
                             const Trajectory& trajectory,
                             const DynamicModelPtr dynamic_model,
                             const State current_state) = 0;

  virtual std::shared_ptr<ExecutionModel> Clone() const = 0;

 private:
  Trajectory last_trajectory_;
  ExecutionStatus execution_status_;
};

typedef std::shared_ptr<ExecutionModel> ExecutionModelPtr;

}  // namespace execution
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_EXECUTION_EXECUTION_MODEL_HPP_
