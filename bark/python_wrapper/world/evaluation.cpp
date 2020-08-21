// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "evaluation.hpp"
#include "bark/world/evaluation/evaluator_behavior_expired.hpp"
#include "bark/world/evaluation/evaluator_collision_agents.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/evaluation/evaluator_step_count.hpp"
#include "bark/world/evaluation/commons.hpp"
#include "bark/world/world.hpp"

#include "bark/python_wrapper/world/ltl.hpp"

namespace py = pybind11;

void python_evaluation(py::module m) {
  using namespace bark::world::evaluation;

  py::class_<BaseEvaluator, PyBaseEvaluator, EvaluatorPtr>(m, "BaseEvaluator")
      .def(py::init<>())
      .def("Evaluate",
           py::overload_cast<const World&>(&BaseEvaluator::Evaluate))
      .def("Evaluate",
           py::overload_cast<const ObservedWorld&>(&BaseEvaluator::Evaluate));

  py::class_<EvaluatorGoalReached, BaseEvaluator,
             std::shared_ptr<EvaluatorGoalReached>>(m, "EvaluatorGoalReached")
      .def(py::init<>())
      .def(py::init<const AgentId&>())
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorGoalReached& g) {
        return "bark.core.world.evaluation.EvaluatorGoalReached";
      });

  py::class_<EvaluatorBehaviorExpired, BaseEvaluator,
             std::shared_ptr<EvaluatorBehaviorExpired>>(
      m, "EvaluatorBehaviorExpired")
      .def(py::init<const AgentId&>())
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorBehaviorExpired& g) {
        return "bark.core.world.evaluation.EvaluatorBehaviorExpired";
      });

  py::class_<EvaluatorCollisionAgents, BaseEvaluator,
             std::shared_ptr<EvaluatorCollisionAgents>>(
      m, "EvaluatorCollisionAgents")
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorCollisionAgents& g) {
        return "bark.core.world.evaluation.EvaluatorCollisionAgents";
      });

  py::class_<EvaluatorDrivableArea, BaseEvaluator,
             std::shared_ptr<EvaluatorDrivableArea>>(m, "EvaluatorDrivableArea")
      .def(py::init<>())
      .def(py::init<const AgentId&>())
      .def("__repr__", [](const EvaluatorDrivableArea& g) {
        return "bark.core.world.evaluation.EvaluatorDrivableArea";
      });

  py::class_<EvaluatorCollisionEgoAgent, BaseEvaluator,
             std::shared_ptr<EvaluatorCollisionEgoAgent>>(
      m, "EvaluatorCollisionEgoAgent")  // NOLINT
      .def(py::init<const AgentId&>())
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorCollisionEgoAgent& g) {
        return "bark.core.world.evaluation.EvaluatorCollisionEgoAgent";
      });
  py::class_<EvaluatorStepCount, BaseEvaluator,
             std::shared_ptr<EvaluatorStepCount>>(m, "EvaluatorStepCount")
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorStepCount& g) {
        return "bark.core.world.evaluation.EvaluatorStepCount";
      });

  m.def("CaptureAgentStates", py::overload_cast<const World&>(
    &CaptureAgentStates<World>));
  m.def("CaptureAgentStates", py::overload_cast<const ObservedWorld&>(
    &CaptureAgentStates<ObservedWorld>));

  python_ltl(m.def_submodule("ltl", "LTL Rules"));
}