// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
#include "python/common.hpp"

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/observed_world.hpp"

namespace py = pybind11;
using namespace modules::models::behavior;
using modules::world::ObservedWorld;
using modules::models::dynamic::Trajectory;

class PyBehaviorModel : public BehaviorModel {
 public:
  using BehaviorModel::BehaviorModel;

  Trajectory Plan(float delta_time,
                  const ObservedWorld& observed_world) override {
    PYBIND11_OVERLOAD_PURE(
        modules::models::dynamic::Trajectory,
        BehaviorModel,
        plan,
        delta_time,
        observed_world);
  }

  std::shared_ptr<BehaviorModel> Clone() const override {
    PYBIND11_OVERLOAD_PURE(
        std::shared_ptr<BehaviorModel>,
        BehaviorModel,
        clone);
  }

};

void python_behavior(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
