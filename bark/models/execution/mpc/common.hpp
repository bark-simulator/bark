// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_MPC_COMMON_HPP_
#define BARK_MODELS_EXECUTION_MPC_COMMON_HPP_

namespace bark {
namespace models {
namespace execution {

struct OptimizationSettings {
  int num_optimization_steps;
  float dt;
};

}  // namespace execution
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_MPC_COMMON_HPP_
