// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_MPC_MPC_HPP_
#define BARK_MODELS_EXECUTION_MPC_MPC_HPP_

#include <Eigen/Core>
#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

#include "bark/models/execution/execution_model.hpp"
#include "bark/models/execution/mpc/common.hpp"
#include "bark/models/execution/mpc/cost_functor.hpp"

namespace bark {
namespace models {
namespace execution {

using dynamic::DynamicModelPtr;
using dynamic::State;
using dynamic::Trajectory;
using Eigen::Dynamic;
using Eigen::Matrix;

class ExecutionModelMpc : public ExecutionModel {
 public:
  explicit ExecutionModelMpc(const commons::ParamsPtr& params);

  ~ExecutionModelMpc() {}

  Matrix<double, Dynamic, Dynamic> get_last_weights() { return last_weights_; }

  Trajectory get_last_desired_states() { return last_desired_states_; }

  void set_last_weights(const Matrix<double, Dynamic, Dynamic>& weights) {
    last_weights_ = weights;
  }

  void set_last_desired_states(const Trajectory& desired_states) {
    last_desired_states_ = desired_states;
  }

  virtual Trajectory Execute(const float& new_world_time,
                             const Trajectory& trajectory,
                             const DynamicModelPtr dynamic_model,
                             const State current_state);

  Trajectory Optimize(
      std::vector<double*> parameter_block, const Trajectory& discrete_behavior,
      const Matrix<double, Dynamic, Dynamic> weights_desired_states);

  virtual std::shared_ptr<ExecutionModel> Clone() const;

 private:
  execution::OptimizationSettings optimization_settings_;
  Matrix<double, Dynamic, Dynamic> last_weights_;
  Trajectory last_desired_states_;
};

inline std::shared_ptr<ExecutionModel> ExecutionModelMpc::Clone() const {
  std::shared_ptr<ExecutionModelMpc> model_ptr =
      std::make_shared<ExecutionModelMpc>(*this);
  return std::dynamic_pointer_cast<ExecutionModel>(model_ptr);
}

}  // namespace execution
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_MPC_MPC_HPP_
