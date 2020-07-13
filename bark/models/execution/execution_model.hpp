// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_EXECUTION_MODEL_HPP_
#define BARK_MODELS_EXECUTION_EXECUTION_MODEL_HPP_

#include <Eigen/Core>
#include <memory>

#include "bark/commons/base_type.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace execution {

using dynamic::DynamicModelPtr;
using dynamic::State;
using dynamic::Trajectory;

enum ExecutionStatus : unsigned int { VALID = 0, INVALID = 1 };

class ExecutionModel : public commons::BaseType {
 public:
  explicit ExecutionModel(bark::commons::ParamsPtr params)
      : BaseType(params),
        last_state_(),
        last_trajectory_(),
        execution_status_(ExecutionStatus::VALID) {}

  ExecutionModel(const ExecutionModel& execution_model)
      : BaseType(execution_model.GetParams()),
        last_state_(execution_model.GetExecutedState()),
        last_trajectory_(execution_model.GetLastTrajectory()),
        execution_status_(execution_model.GetExecutionStatus()) {}

  virtual ~ExecutionModel() {}

  State GetExecutedState() const { return last_state_; }
  Trajectory GetLastTrajectory() const { return last_trajectory_; }

  void SetLastState(const State& state) { last_state_ = state; }

  void SetLastTrajectory(const Trajectory& traj) { last_trajectory_ = traj; }

  void SetExecutionStatus(const ExecutionStatus& execution_status) {
    execution_status_ = execution_status;
  }

  ExecutionStatus GetExecutionStatus() const { return execution_status_; }

  virtual void Execute(const float& new_world_time,
                       const Trajectory& trajectory,
                       const DynamicModelPtr dynamic_model) = 0;

  virtual std::shared_ptr<ExecutionModel> Clone() const = 0;

 private:
  State last_state_;
  Trajectory last_trajectory_;
  ExecutionStatus execution_status_;
};

typedef std::shared_ptr<ExecutionModel> ExecutionModelPtr;

}  // namespace execution
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_EXECUTION_MODEL_HPP_
