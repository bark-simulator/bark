// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/python_wrapper/world/ltl.hpp"

#include "bark/python_wrapper/world/evaluation.hpp"

#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/ltl/evaluator_being_overtaken.hpp"
#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"
#include "bark/world/evaluation/ltl/evaluator_right_overtake.hpp"
#include "bark/world/evaluation/ltl/evaluator_safe_distance.hpp"
#include "bark/world/evaluation/ltl/evaluator_safe_lane_change.hpp"
#include "bark/world/evaluation/ltl/evaluator_speed_advantage_overtake.hpp"
#include "bark/world/evaluation/ltl/evaluator_zip_merge_chn.hpp"
#include "bark/world/evaluation/ltl/evaluator_zip_merge_deu.hpp"
#include "bark/world/evaluation/ltl/labels/agent_at_lane_end_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/agent_beyond_point_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/agent_near_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/base_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/behind_of_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/ego_accelerate_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/ego_beyond_point_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/front_of_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/generic_ego_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/lane_change_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/left_of_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/preceding_agent_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/rel_speed_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/right_of_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/safe_distance_label_function.hpp"
#include "bark/world/tests/evaluation/constant_label_evaluator.hpp"

namespace py = pybind11;

void python_ltl(py::module m) {
  using namespace modules::world::evaluation;

  py::class_<EvaluatorLTL, BaseEvaluator, std::shared_ptr<EvaluatorLTL>>(
      m, "EvaluatorLTL")
      .def(py::init<AgentId, const std::string &, const LabelFunctions &>())
      .def_property_readonly("rule_states", &EvaluatorLTL::GetRuleStates)
      .def("__repr__", [](const EvaluatorLTL &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorLTL";
      });

  py::class_<EvaluatorRightOvertake, BaseEvaluator,
             std::shared_ptr<EvaluatorRightOvertake>>(m,
                                                      "EvaluatorRightOvertake")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorRightOvertake::GetRuleStates)
      .def("__repr__", [](const EvaluatorRightOvertake &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorRightOvertake";
      });

  py::class_<EvaluatorSafeDistance, BaseEvaluator,
             std::shared_ptr<EvaluatorSafeDistance>>(m, "EvaluatorSafeDistance")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorSafeDistance::GetRuleStates)
      .def("__repr__", [](const EvaluatorSafeDistance &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorSafeDistance";
      });

  py::class_<EvaluatorSafeLaneChange, BaseEvaluator,
             std::shared_ptr<EvaluatorSafeLaneChange>>(
      m, "EvaluatorSafeLaneChange")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorSafeLaneChange::GetRuleStates)
      .def("__repr__", [](const EvaluatorSafeLaneChange &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorSafeLaneChange";
      });

  py::class_<EvaluatorSafeLaneChangeAssumption, BaseEvaluator,
             std::shared_ptr<EvaluatorSafeLaneChangeAssumption>>(
      m, "EvaluatorSafeLaneChangeAssumption")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorSafeLaneChangeAssumption::GetRuleStates)
      .def("__repr__", [](const EvaluatorSafeLaneChangeAssumption &g) {
        return "bark.core.world.evaluation.ltl."
               "EvaluatorSafeLaneChangeAssumption";
      });

  py::class_<EvaluatorBeingOvertaken, BaseEvaluator,
             std::shared_ptr<EvaluatorBeingOvertaken>>(
      m, "EvaluatorBeingOvertaken")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorBeingOvertaken::GetRuleStates)
      .def("__repr__", [](const EvaluatorBeingOvertaken &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorBeingOvertaken";
      });

  py::class_<EvaluatorBeingOvertakenAssumption, BaseEvaluator,
             std::shared_ptr<EvaluatorBeingOvertakenAssumption>>(
      m, "EvaluatorBeingOvertakenAssumption")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorBeingOvertakenAssumption::GetRuleStates)
      .def("__repr__", [](const EvaluatorBeingOvertakenAssumption &g) {
        return "bark.core.world.evaluation.ltl."
               "EvaluatorBeingOvertakenAssumption";
      });

  py::class_<EvaluatorSpeedAdvantageOvertake, BaseEvaluator,
             std::shared_ptr<EvaluatorSpeedAdvantageOvertake>>(
      m, "EvaluatorSpeedAdvantageOvertake")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorSpeedAdvantageOvertake::GetRuleStates)
      .def("__repr__", [](const EvaluatorSpeedAdvantageOvertake &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorSpeedAdvantageOvertake";
      });

  py::class_<EvaluatorZipMergeDeu, BaseEvaluator,
             std::shared_ptr<EvaluatorZipMergeDeu>>(m, "EvaluatorZipMergeDeu")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorZipMergeDeu::GetRuleStates)
      .def("__repr__", [](const EvaluatorZipMergeDeu &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorZipMergeDeu";
      });

  py::class_<EvaluatorZipMergeChn, BaseEvaluator,
             std::shared_ptr<EvaluatorZipMergeChn>>(m, "EvaluatorZipMergeChn")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorZipMergeChn::GetRuleStates)
      .def("__repr__", [](const EvaluatorZipMergeChn &g) {
        return "bark.core.world.evaluation.ltl.EvaluatorZipMergeChn";
      });

  py::class_<EvaluatorRightOvertakeAssumption, BaseEvaluator,
             std::shared_ptr<EvaluatorRightOvertakeAssumption>>(
      m, "EvaluatorRightOvertakeAssumption")
      .def(py::init<AgentId>())
      .def_property_readonly("rule_states",
                             &EvaluatorRightOvertakeAssumption::GetRuleStates)
      .def("__repr__", [](const EvaluatorRightOvertakeAssumption &g) {
        return "bark.core.world.evaluation.ltl."
               "EvaluatorRightOvertakeAssumption";
      });

  // LABELS

  py::class_<BaseLabelFunction, PyBaseLabelFunction,
             std::shared_ptr<BaseLabelFunction>>(m, "BaseLabelFunction")
      .def(py::init<const std::string &>())
      .def("Evaluate", &BaseLabelFunction::Evaluate);

  py::class_<ConstantLabelFunction, BaseLabelFunction,
             std::shared_ptr<ConstantLabelFunction>>(m, "ConstantLabelFunction")
      .def(py::init<const std::string &>())
      .def_property("value", &ConstantLabelFunction::GetValue,
                    &ConstantLabelFunction::SetValue);

  py::class_<SafeDistanceLabelFunction, BaseLabelFunction,
             std::shared_ptr<SafeDistanceLabelFunction>>(
      m, "SafeDistanceLabelFunction")
      .def(py::init<const std::string &, bool, double, double, double>());

  py::class_<LaneChangeLabelFunction, BaseLabelFunction,
             std::shared_ptr<LaneChangeLabelFunction>>(
      m, "LaneChangeLabelFunction")
      .def(py::init<const std::string &>());

  py::class_<AgentNearLabelFunction, BaseLabelFunction,
             std::shared_ptr<AgentNearLabelFunction>>(m,
                                                      "AgentNearLabelFunction")
      .def(py::init<const std::string &, double>());

  py::class_<EgoAccelerateLabelFunction, BaseLabelFunction,
             std::shared_ptr<EgoAccelerateLabelFunction>>(
      m, "EgoAccelerateLabelFunction")
      .def(py::init<const std::string &, double>());

  py::class_<RelSpeedLabelFunction, BaseLabelFunction,
             std::shared_ptr<RelSpeedLabelFunction>>(m, "RelSpeedLabelFunction")
      .def(py::init<const std::string &, double>());

  py::class_<AgentAtLaneEndLabelFunction, BaseLabelFunction,
             std::shared_ptr<AgentAtLaneEndLabelFunction>>(
      m, "AgentAtLaneEndLabelFunction")
      .def(py::init<const std::string &, double>());

  py::class_<RightOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<RightOfLabelFunction>>(m, "RightOfLabelFunction")
      .def(py::init<const std::string &>());

  py::class_<LeftOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<LeftOfLabelFunction>>(m, "LeftOfLabelFunction")
      .def(py::init<const std::string &>());

  py::class_<BehindOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<BehindOfLabelFunction>>(m, "BehindOfLabelFunction")
      .def(py::init<const std::string &>());

  py::class_<FrontOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<FrontOfLabelFunction>>(m, "FrontOfLabelFunction")
      .def(py::init<const std::string &>());

  py::class_<AgentBeyondPointLabelFunction, BaseLabelFunction,
             std::shared_ptr<AgentBeyondPointLabelFunction>>(
      m, "AgentBeyondPointLabelFunction")
      .def(py::init<const std::string &, const Point2d &>())
      .def(py::pickle(
          [](const AgentBeyondPointLabelFunction &b) {
            return py::make_tuple(b.GetLabelStr(), b.GetBeyondPoint());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new AgentBeyondPointLabelFunction(t[0].cast<std::string>(),
                                                     t[1].cast<Point2d>());
          }));

  py::class_<EgoBeyondPointLabelFunction, BaseLabelFunction,
             std::shared_ptr<EgoBeyondPointLabelFunction>>(
      m, "EgoBeyondPointLabelFunction")
      .def(py::init<const std::string &, const Point2d &>())
      .def(py::pickle(
          [](const EgoBeyondPointLabelFunction &b) {
            return py::make_tuple(b.GetLabelStr(), b.GetBeyondPoint());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoBeyondPointLabelFunction(t[0].cast<std::string>(),
                                                   t[1].cast<Point2d>());
          }));

  py::class_<PrecedingAgentLabelFunction, BaseLabelFunction,
             std::shared_ptr<PrecedingAgentLabelFunction>>(
      m, "PrecedingAgentLabelFunction")
      .def(py::init<const std::string &>())
      .def(py::pickle(
          [](const PrecedingAgentLabelFunction &b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new PrecedingAgentLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<
      GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>, BaseLabelFunction,
      std::shared_ptr<GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>>>(
      m, "CollisionEgoLabelFunction")
      .def(py::init<const std::string &>())
      .def(py::pickle(
          [](const GenericEgoLabelFunction<EvaluatorCollisionEgoAgent> &b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>(
                t[0].cast<std::string>());
          }));
}