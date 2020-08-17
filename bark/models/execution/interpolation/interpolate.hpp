// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_INTERPOLATION_INTERPOLATE_HPP_
#define BARK_MODELS_EXECUTION_INTERPOLATION_INTERPOLATE_HPP_

#include <Eigen/Core>
#include "bark/models/execution/execution_model.hpp"

namespace bark {
namespace models {
namespace execution {

using bark::commons::ParamsPtr;
using dynamic::State;

class ExecutionModelInterpolate : public ExecutionModel {
 public:
  explicit ExecutionModelInterpolate(const ParamsPtr& params)
      : ExecutionModel(params) {}
  ~ExecutionModelInterpolate() {}

  /**
   * @brief  Checks if the world time is within the trajectory
   * @note
   * @retval boolean: true if contained
   */
  bool CheckIfWorldTimeIsWithinTrajectory(const Trajectory& trajectory,
                                          const float& world_time) const;

  /**
   * @brief  Find exact time in trajectory
   * @retval BARK state and whether is was found
   */
  std::pair<State, bool> CheckIfTimeExactIsInTrajectory(
      const Trajectory& trajectory, const double& world_time) const;

  /**
   * @brief  Find lower time point in trajectory for world_time
   * @retval Trajectory row-id and whether it was found
   */
  std::pair<int, bool> FindClosestLowerTrajectoryRow(
      const Trajectory& trajectory, const double& world_time) const;

  /**
   * @brief  Interpolates between two states
   * @retval State: interpolated state
   */
  State Interpolate(const State& p0, const State& p1, const float& time) const;

  /**
   * @brief  Interpolates on trajectory
   */
  virtual void Execute(const float& new_world_time,
                       const dynamic::Trajectory& trajectory,
                       const dynamic::DynamicModelPtr dynamic_model);

  virtual std::shared_ptr<ExecutionModel> Clone() const;
};

inline std::shared_ptr<ExecutionModel> ExecutionModelInterpolate::Clone()
    const {
  std::shared_ptr<ExecutionModelInterpolate> model_ptr =
      std::make_shared<ExecutionModelInterpolate>(*this);
  return std::dynamic_pointer_cast<ExecutionModel>(model_ptr);
}

}  // namespace execution
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_INTERPOLATION_INTERPOLATE_HPP_
