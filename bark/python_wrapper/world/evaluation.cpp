// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "evaluation.hpp"
#include "bark/world/evaluation/commons.hpp"
#include "bark/world/evaluation/evaluator_behavior_expired.hpp"
#include "bark/world/evaluation/evaluator_collision_agents.hpp"
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/evaluator_drivable_area.hpp"
#include "bark/world/evaluation/evaluator_goal_reached.hpp"
#include "bark/world/evaluation/evaluator_step_count.hpp"
#include "bark/world/world.hpp"

#include "bark/python_wrapper/world/ltl.hpp"
#include "bark/world/evaluation/rss/safety_polygon.hpp"
#ifdef RSS
#include "bark/world/evaluation/rss/evaluator_rss.hpp"
#endif

namespace py = pybind11;

void python_evaluation(py::module m) {
  using namespace bark::world::evaluation;
  using geometry::Polygon;

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

  py::class_<SafetyPolygon,
             std::shared_ptr<SafetyPolygon>>(m, "SafetyPolygon")
    .def(py::init<>())
    .def("__repr__", [](const SafetyPolygon& g) {
      return "bark.core.world.evaluation.SafetyPolygon";
  })
  .def(py::pickle(
    [](const SafetyPolygon& a) {
      // make tuple here
      return py::make_tuple(
        a.lat_left_safety_distance, a.lat_right_safety_distance,
        a.lon_safety_distance, a.polygon);
    },
    [](py::tuple t) {
      if (t.size() != 4)
        throw std::runtime_error("Invalid SafetyPolygon model state!");
      return new SafetyPolygon{
        t[0].cast<double>(), t[1].cast<double>(),
        t[2].cast<double>(), t[3].cast<Polygon>()};
  }));


#ifdef RSS
  py::class_<EvaluatorRSS, BaseEvaluator, std::shared_ptr<EvaluatorRSS>>(
      m, "EvaluatorRSS")
      .def(py::init<const bark::commons::ParamsPtr>(), py::arg("params"))
      .def(py::init<const AgentId&, const bark::commons::ParamsPtr>(),
           py::arg("agent_id"), py::arg("params"))
      .def("Evaluate", py::overload_cast<const World&>(&EvaluatorRSS::Evaluate))
      .def("PairwiseEvaluate",
           py::overload_cast<const World&>(&EvaluatorRSS::PairwiseEvaluate))
      .def("PairwiseDirectionalEvaluate",
           py::overload_cast<const World&>(
               &EvaluatorRSS::PairwiseDirectionalEvaluate))
      .def("__repr__", [](const EvaluatorRSS& g) {
        return "bark.core.world.evaluation.EvaluatorRSS";
      });
#endif

  m.def("CaptureAgentStates",
        py::overload_cast<const World&>(&CaptureAgentStates<World>));
  m.def("CaptureAgentStates", py::overload_cast<const ObservedWorld&>(
                                  &CaptureAgentStates<ObservedWorld>));

  python_ltl(m.def_submodule("ltl", "LTL Rules"));
}