// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "evaluation.hpp"
#include "modules/world/evaluation/evaluator_behavior_expired.hpp"
#include "modules/world/evaluation/evaluator_collision_agents.hpp"
#include "modules/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "modules/world/evaluation/evaluator_drivable_area.hpp"
#include "modules/world/evaluation/evaluator_goal_reached.hpp"
#include "modules/world/evaluation/evaluator_ltl.hpp"
#include "modules/world/evaluation/evaluator_right_overtake.hpp"
#include "modules/world/evaluation/evaluator_safe_distance.hpp"
#include "modules/world/evaluation/evaluator_step_count.hpp"
#include "modules/world/evaluation/labels/agent_beyond_point_label_evaluator.hpp"
#include "modules/world/evaluation/labels/base_label_evaluator.hpp"
#include "modules/world/evaluation/labels/behind_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/direct_front_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/ego_beyond_point_label_evaluator.hpp"
#include "modules/world/evaluation/labels/front_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/generic_ego_label_evaluator.hpp"
#include "modules/world/evaluation/labels/left_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/right_of_label_evaluator.hpp"
#include "modules/world/evaluation/labels/safe_distance_label_evaluator.hpp"
#include "modules/world/tests/constant_label_evaluator.hpp"
#include "modules/world/world.hpp"

namespace py = pybind11;

void python_evaluation(py::module m) {
  using namespace modules::world::evaluation;

  py::class_<BaseEvaluator,
             PyBaseEvaluator,
             EvaluatorPtr>(m, "BaseEvaluator")
      .def(py::init<>())
      .def("Evaluate",
        py::overload_cast<const World&>(&BaseEvaluator::Evaluate))
      .def("Evaluate",
        py::overload_cast<const ObservedWorld&>(&BaseEvaluator::Evaluate));

  py::class_<EvaluatorGoalReached, BaseEvaluator,
      std::shared_ptr<EvaluatorGoalReached> >(m, "EvaluatorGoalReached")
      .def(py::init<>())
      .def(py::init<const AgentId&>())
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorGoalReached &g) {
        return "bark.world.evaluation.EvaluatorGoalReached";
      });

  py::class_<EvaluatorBehaviorExpired, BaseEvaluator,
      std::shared_ptr<EvaluatorBehaviorExpired> >(m, "EvaluatorBehaviorExpired")
      .def(py::init<const AgentId&>())
      .def(py::init<>())
      .def("__repr__", [](const EvaluatorBehaviorExpired &g) {
        return "bark.world.evaluation.EvaluatorBehaviorExpired";
      });

  py::class_<EvaluatorCollisionAgents, BaseEvaluator,
    std::shared_ptr<EvaluatorCollisionAgents> >(m, "EvaluatorCollisionAgents")
    .def(py::init<>())
    .def("__repr__", [](const EvaluatorCollisionAgents &g) {
      return "bark.world.evaluation.EvaluatorCollisionAgents";
    });

  py::class_<EvaluatorDrivableArea, BaseEvaluator,
    std::shared_ptr<EvaluatorDrivableArea> >(m, "EvaluatorDrivableArea")
    .def(py::init<>())
    .def(py::init<const AgentId&>())
    .def("__repr__", [](const EvaluatorDrivableArea &g) {
      return "bark.world.evaluation.EvaluatorDrivableArea";
    });

  py::class_<EvaluatorCollisionEgoAgent, BaseEvaluator,
    std::shared_ptr<EvaluatorCollisionEgoAgent>>(m, "EvaluatorCollisionEgoAgent")  // NOLINT
    .def(py::init<const AgentId&>())
    .def(py::init<>())
    .def("__repr__", [](const EvaluatorCollisionEgoAgent &g) {
      return "bark.world.evaluation.EvaluatorCollisionEgoAgent";
    });
  py::class_<EvaluatorStepCount, BaseEvaluator,
    std::shared_ptr<EvaluatorStepCount> >(m, "EvaluatorStepCount")
    .def(py::init<>())
    .def("__repr__", [](const EvaluatorStepCount &g) {
      return "bark.world.evaluation.EvaluatorStepCount";
    });

  py::class_<EvaluatorLTL, BaseEvaluator, std::shared_ptr<EvaluatorLTL>>(m, "EvaluatorLTL")
      .def(py::init<AgentId, const std::string&>())
      .def_property_readonly("rule_states", &EvaluatorLTL::GetRuleStates)
      .def("__repr__", [](const EvaluatorLTL &g) {
        return "bark.world.evaluation.EvaluatorLTL";
      });

  py::class_<EvaluatorRightOvertake, BaseEvaluator,
      std::shared_ptr<EvaluatorRightOvertake>>(m, "EvaluatorRightOvertake")
          .def(py::init<AgentId>())
          .def_property_readonly("rule_states", &EvaluatorRightOvertake::GetRuleStates)
          .def("__repr__", [](const EvaluatorRightOvertake &g) {
            return "bark.world.evaluation.EvaluatorRightOvertake";
          });
  py::class_<EvaluatorSafeDistance, BaseEvaluator,
             std::shared_ptr<EvaluatorSafeDistance>>(m, "EvaluatorSafeDistance")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorSafeDistance::GetRuleStates)
      .def("__repr__", [](const EvaluatorSafeDistance &g) {
        return "bark.world.evaluation.EvaluatorSafeDistance";
      });

  // LABELS

  py::class_<BaseLabelEvaluator, PyBaseLabelEvaluator,
             std::shared_ptr<BaseLabelEvaluator>>(m, "BaseLabelEvaluator")
      .def(py::init<const std::string &>())
      .def("Evaluate", &BaseLabelEvaluator::Evaluate);

  py::class_<ConstantLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<ConstantLabelEvaluator>>(m, "ConstantLabelEvaluator")
      .def(py::init<const std::string &>())
      .def_property("value", &ConstantLabelEvaluator::GetValue, &ConstantLabelEvaluator::SetValue);

  py::class_<SafeDistanceLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<SafeDistanceLabelEvaluator>>(m, "SafeDistanceLabelEvaluator")
      .def(py::init<const std::string &, bool, double, double, double>());

  py::class_<RightOfLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<RightOfLabelEvaluator>>(m, "RightOfLabelEvaluator")
      .def(py::init<const std::string &>());

  py::class_<LeftOfLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<LeftOfLabelEvaluator>>(m, "LeftOfLabelEvaluator")
      .def(py::init<const std::string &>());

  py::class_<BehindOfLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<BehindOfLabelEvaluator>>(m,
                                                      "BehindOfLabelEvaluator")
      .def(py::init<const std::string &>());

  py::class_<FrontOfLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<FrontOfLabelEvaluator>>(m, "FrontOfLabelEvaluator")
      .def(py::init<const std::string &>());

  py::class_<AgentBeyondPointLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<AgentBeyondPointLabelEvaluator>>(
      m, "AgentBeyondPointLabelEvaluator")
      .def(py::init<const std::string &, const Point2d &>())
      .def(py::pickle(
          [](const AgentBeyondPointLabelEvaluator &b) {
            return py::make_tuple(b.GetLabelStr(), b.GetBeyondPoint());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new AgentBeyondPointLabelEvaluator(t[0].cast<std::string>(),
                                                      t[1].cast<Point2d>());
          }));

  py::class_<EgoBeyondPointLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<EgoBeyondPointLabelEvaluator>>(
      m, "EgoBeyondPointLabelEvaluator")
      .def(py::init<const std::string &, const Point2d &>())
      .def(py::pickle(
          [](const EgoBeyondPointLabelEvaluator &b) {
            return py::make_tuple(b.GetLabelStr(), b.GetBeyondPoint());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoBeyondPointLabelEvaluator(t[0].cast<std::string>(),
                                                    t[1].cast<Point2d>());
          }));

  py::class_<DirectFrontOfLabelEvaluator, BaseLabelEvaluator,
             std::shared_ptr<DirectFrontOfLabelEvaluator>>(
      m, "DirectFrontOfLabelEvaluator")
      .def(py::init<const std::string &>())
      .def(py::pickle(
          [](const DirectFrontOfLabelEvaluator &b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new DirectFrontOfLabelEvaluator(t[0].cast<std::string>());
          }));

  py::class_<
  GenericEgoLabelEvaluator<EvaluatorCollisionEgoAgent>, BaseLabelEvaluator,
      std::shared_ptr<GenericEgoLabelEvaluator<EvaluatorCollisionEgoAgent>>>(
      m, "CollisionEgoLabelEvaluator")
      .def(py::init<const std::string &>())
      .def(py::pickle(
          [](const GenericEgoLabelEvaluator<EvaluatorCollisionEgoAgent> &b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new GenericEgoLabelEvaluator<EvaluatorCollisionEgoAgent>(
                t[0].cast<std::string>());
          }));
}