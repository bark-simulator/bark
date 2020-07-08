// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
#include <memory>
#include "bark/python_wrapper/common.hpp"

#include "bark/models/behavior/behavior_model.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/observed_world.hpp"

namespace py = pybind11;
using bark::models::behavior::Action;
using bark::models::behavior::BehaviorModel;
using bark::models::behavior::primitives::Primitive;
using bark::models::dynamic::DynamicModelPtr;
using bark::models::dynamic::Trajectory;
using bark::world::ObservedWorld;
using bark::world::ObservedWorldPtr;

class PyBehaviorModel : public BehaviorModel {
 public:
  using BehaviorModel::BehaviorModel;

  Trajectory Plan(float min_planning_time,
                  const ObservedWorld& observed_world) {
    PYBIND11_OVERLOAD_PURE(bark::models::dynamic::Trajectory, BehaviorModel,
                           Plan, min_planning_time, observed_world);
  }

  std::shared_ptr<BehaviorModel> Clone() const {
    PYBIND11_OVERLOAD_PURE(std::shared_ptr<BehaviorModel>, BehaviorModel,
                           Clone);
  }

  void ActionToBehavior(const Action& action) {
    PYBIND11_OVERLOAD(void, BehaviorModel, ActionToBehavior, action);
  }
};

class PyPrimitive : public Primitive {
 public:
  using Primitive::Primitive;

  bool IsPreConditionSatisfied(
      const ObservedWorld& observed_world,
      const bark::models::behavior::primitives::AdjacentLaneCorridors&
          adjacent_corridors) {
    PYBIND11_OVERLOAD_PURE(bool, Primitive, IsPreConditionSatisfied,
                           observed_world, adjacent_corridors);
  }

  Trajectory Plan(float min_planning_time, const ObservedWorld& observed_world,
                  const bark::world::LaneCorridorPtr& target_corridor) {
    PYBIND11_OVERLOAD_PURE(bark::models::dynamic::Trajectory, Primitive, Plan,
                           min_planning_time, observed_world, target_corridor);
  }

  bark::world::LaneCorridorPtr SelectTargetCorridor(
      const ObservedWorld& observed_world,
      const bark::models::behavior::primitives::AdjacentLaneCorridors&
          adjacent_corridors) {
    PYBIND11_OVERLOAD_PURE(bark::world::LaneCorridorPtr, Primitive,
                           SelectTargetCorridor, observed_world,
                           adjacent_corridors);
  }
};

void python_behavior(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_BEHAVIOR_HPP_
