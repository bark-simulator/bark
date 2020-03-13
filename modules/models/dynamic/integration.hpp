// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_DYNAMIC_INTEGRATION_HPP_
#define MODULES_MODELS_DYNAMIC_INTEGRATION_HPP_
#include <Eigen/Core>

#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace dynamic {

inline State euler_int(const DynamicModel& model,
                const State &x,
                const Input &u,
                float dt) {
  return x + dt * model.StateSpaceModel(x, u);
}

inline State rk4(const DynamicModel& model, const State &x, const Input &u, float dt) {
  return x;
}

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_INTEGRATION_HPP_
