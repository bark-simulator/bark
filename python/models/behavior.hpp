// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
#include "python/common.hpp"

#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/models/behavior/motion_primitives/primitives.hpp"
#include "modules/world/observed_world.hpp"


namespace py = pybind11;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::primitives::Primitive;
using modules::world::ObservedWorld;
using modules::world::ObservedWorldPtr;
using modules::models::dynamic::Trajectory;
using modules::models::dynamic::DynamicModelPtr;

class PyBehaviorModel : public BehaviorModel {
 public:
  using BehaviorModel::BehaviorModel;

  Trajectory Plan(float delta_time,
                  const ObservedWorld& observed_world) {
    PYBIND11_OVERLOAD_PURE(
      modules::models::dynamic::Trajectory,
      BehaviorModel,
      Plan,
      delta_time,
      observed_world);
  }

  std::shared_ptr<BehaviorModel> Clone() const {
    PYBIND11_OVERLOAD(
      std::shared_ptr<BehaviorModel>,
      BehaviorModel,
      Clone);
  }

};

class PyPrimitive : public Primitive {
 public:
  using Primitive::Primitive;

  bool IsPreConditionSatisfied(
      const ObservedWorldPtr& observed_world) {
        PYBIND11_OVERLOAD_PURE(
      bool,
      Primitive,
      IsPreConditionSatisfied,
      observed_world);
  }

  Trajectory Plan(float delta_time,
                          const ObservedWorld& observed_world) {
      PYBIND11_OVERLOAD_PURE(
      modules::models::dynamic::Trajectory,
      Primitive,
      Plan,
      delta_time,
      observed_world);
    }
};

void python_behavior(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_

