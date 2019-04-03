// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_EXECUTION_MPC_COMMON_HPP_
#define MODULES_MODELS_EXECUTION_MPC_COMMON_HPP_

namespace modules {
namespace models {
namespace execution {

struct OptimizationSettings {
  int num_optimization_steps;
  float dt;
};

}  // namespace execution
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_EXECUTION_MPC_COMMON_HPP_
