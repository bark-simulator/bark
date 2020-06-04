// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.



#ifndef MODULES_MODELS_EXECUTION_INTERPOLATION_INTERPOLATE_HPP_
#define MODULES_MODELS_EXECUTION_INTERPOLATION_INTERPOLATE_HPP_

#include <Eigen/Core>
#include "modules/models/execution/execution_model.hpp"

namespace modules {
namespace models {
namespace execution {

using dynamic::State;
using modules::commons::ParamsPtr;

class ExecutionModelInterpolate : public ExecutionModel {
 public:
  explicit ExecutionModelInterpolate(
    const ParamsPtr& params) : ExecutionModel(params) {}
  ~ExecutionModelInterpolate() {}

  bool CheckIfWorldTimeIsWithinTrajectory(
    const Trajectory& trajectory,
    const float& world_time) const;

  std::pair<State, bool> CheckIfTimeExactIsInTrajectory(
    const Trajectory& trajectory,
    const double& world_time) const;
  
  virtual void Execute(
    const float &new_world_time,
    const dynamic::Trajectory& trajectory,
    const dynamic::DynamicModelPtr dynamic_model);

  virtual std::shared_ptr<ExecutionModel> Clone() const;
};

inline std::shared_ptr<ExecutionModel> ExecutionModelInterpolate::Clone() const {
  std::shared_ptr<ExecutionModelInterpolate> model_ptr =
    std::make_shared<ExecutionModelInterpolate>(*this);
  return std::dynamic_pointer_cast<ExecutionModel>(model_ptr);
}


}  // namespace execution
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_EXECUTION_INTERPOLATION_INTERPOLATE_HPP_
