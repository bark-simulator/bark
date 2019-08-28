// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
#define PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
#include "python/common.hpp"
#include "modules/runtime/runtime.hpp"

namespace py = pybind11;
using modules::world::ObservedWorld;
using modules::models::dynamic::Trajectory;

class PyRuntime : public Runtime {
 public:
  Trajectory Plan(float delta_time,
                  const ObservedWorld& observed_world) override {
    PYBIND11_OVERLOAD_PURE(
        modules::models::dynamic::Trajectory,
        BehaviorModel,
        plan,
        delta_time,
        observed_world);
  }
};

void python_runtime(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_RUNTIME_HPP_
