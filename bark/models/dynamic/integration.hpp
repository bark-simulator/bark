// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_DYNAMIC_INTEGRATION_HPP_
#define BARK_MODELS_DYNAMIC_INTEGRATION_HPP_
#include <Eigen/Core>

#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace dynamic {

inline State euler_int(const DynamicModel& model, const State& x,
                       const Input& u, float dt) {
  return x + dt * model.StateSpaceModel(x, u);
}

inline State rk4(const DynamicModel& model, const State& x, const Input& u,
                 float dt) {
  State k0 = dt * model.StateSpaceModel(x, u);
  State k1 = dt * model.StateSpaceModel(x + k0 / 2, u);
  State k2 = dt * model.StateSpaceModel(x + k1 / 2, u);
  State k3 = dt * model.StateSpaceModel(x + k2, u);
  return x + 1.0 / 6.0 * (k0 + 2 * k1 + 2 * k2 + k3);
}

}  // namespace dynamic
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_DYNAMIC_INTEGRATION_HPP_
