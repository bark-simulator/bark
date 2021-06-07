// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/python_wrapper/world/ltl.hpp"

#include "bark/python_wrapper/world/evaluation.hpp"

#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/evaluation/ltl/evaluator_ltl.hpp"
#include "bark/world/evaluation/ltl/label_functions/agent_at_lane_end_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/agent_beyond_point_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/agent_near_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/base_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/behind_of_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/below_speed_limit_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/constant_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/dense_traffic_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/ego_accelerate_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/ego_below_speed_limit_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/ego_beyond_point_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/ego_leftmost_lane_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/ego_rightmost_lane_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/front_of_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/generic_ego_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/lane_change_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/left_of_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/succeeding_agent_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/rel_speed_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/right_of_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/rightmost_lane_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/on_road_label_function.hpp"
#include "bark/world/evaluation/ltl/label_functions/safe_distance_label_function.hpp"

namespace py = pybind11;

void python_ltl(py::module m) {
  using namespace bark::world::evaluation;

#ifdef LTL_RULES
  py::class_<EvaluatorLTL, BaseEvaluator, std::shared_ptr<EvaluatorLTL>>(
      m, "EvaluatorLTL")
      .def(py::init<AgentId, const std::string&, const LabelFunctions&>(),
           py::arg("agent_id"), py::arg("ltl_formula"),
           py::arg("label_functions"))
      .def_property_readonly("rule_states", &EvaluatorLTL::GetRuleStates)
      .def_property_readonly("label_functions",
                             &EvaluatorLTL::GetLabelFunctions)
      .def_property_readonly("safety_violations", &EvaluatorLTL::GetSafetyViolations)
      .def("__repr__", [](const EvaluatorLTL& g) {
        return "bark.core.world.evaluation.ltl.EvaluatorLTL";
      });
#endif
  // LABELS

  py::class_<BaseLabelFunction, PyBaseLabelFunction,
             std::shared_ptr<BaseLabelFunction>>(m, "BaseLabelFunction")
      .def(py::init<const std::string&>())
      .def("Evaluate", &BaseLabelFunction::Evaluate);

  py::class_<ConstantLabelFunction, BaseLabelFunction,
             std::shared_ptr<ConstantLabelFunction>>(m, "ConstantLabelFunction")
      .def(py::init<const std::string&>())
      .def_property("value", &ConstantLabelFunction::GetValue,
                    &ConstantLabelFunction::SetValue)
      .def("__repr__",
           [](const ConstantLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.ConstantLabelFunction";
           })
      .def(py::pickle(
          [](const ConstantLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new ConstantLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<SafeDistanceLabelFunction, BaseLabelFunction,
             std::shared_ptr<SafeDistanceLabelFunction>>(
      m, "SafeDistanceLabelFunction")
      .def(py::init<const std::string&, bool, double, double, double, double, bool,
                    unsigned int, bool, double, bool>())
      .def("__repr__",
           [](const SafeDistanceLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.SafeDistanceLabelFunction";
           })
      .def(py::pickle(
          [](const SafeDistanceLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetToRear(), b.GetDeltaEgo(),
                                  b.GetDeltaOthers(), b.GetMaxDecelEgo(),
                                  b.GetMaxDecelOther(), 
                                  b.GetConsiderCrossingCorridors(),
                                  b.GetMaxAgentsForCrossing(),
                                  b.GetUseFracLateralOffsetParam(),
                                  b.GetLateralDifferenceThreshold(),
                                  b.GetCheckLateralDist());
          },
          [](py::tuple t) {
            if (t.size() != 7)
              throw std::runtime_error("Invalid label evaluator state!");
            return new SafeDistanceLabelFunction(
                t[0].cast<std::string>(), t[1].cast<bool>(),
                t[2].cast<double>(), t[3].cast<double>(), t[4].cast<double>(),
                t[5].cast<double>(), t[6].cast<bool>(), t[7].cast<unsigned int>(),
                t[8].cast<bool>(), t[9].cast<double>(), t[10].cast<bool>());
          }));

  py::class_<BelowSpeedLimitLabelFunction, BaseLabelFunction,
             std::shared_ptr<BelowSpeedLimitLabelFunction>>(
      m, "BelowSpeedLimitLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const BelowSpeedLimitLabelFunction& g) {
             return "bark.core.world.evaluation.ltl."
                    "BelowSpeedLimitLabelFunction";
           })
      .def(py::pickle(
          [](const BelowSpeedLimitLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetVelocityThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new BelowSpeedLimitLabelFunction(t[0].cast<std::string>(),
                                                    t[1].cast<double>());
          }));

  py::class_<EgoBelowSpeedLimitLabelFunction, BaseLabelFunction,
             std::shared_ptr<EgoBelowSpeedLimitLabelFunction>>(
      m, "EgoBelowSpeedLimitLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const EgoBelowSpeedLimitLabelFunction& g) {
             return "bark.core.world.evaluation.ltl."
                    "EgoBelowSpeedLimitLabelFunction";
           })
      .def(py::pickle(
          [](const EgoBelowSpeedLimitLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetVelocityThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoBelowSpeedLimitLabelFunction(t[0].cast<std::string>(),
                                                       t[1].cast<double>());
          }));

  py::class_<LaneChangeLabelFunction, BaseLabelFunction,
             std::shared_ptr<LaneChangeLabelFunction>>(
      m, "LaneChangeLabelFunction")
      .def(py::init<const std::string&>())
      .def("__repr__",
           [](const LaneChangeLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.LaneChangeLabelFunction";
           })
      .def(py::pickle(
          [](const LaneChangeLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new LaneChangeLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<DenseTrafficLabelFunction, BaseLabelFunction,
             std::shared_ptr<DenseTrafficLabelFunction>>(
      m, "DenseTrafficLabelFunction")
      .def(py::init<const std::string&, double, int>())
      .def("__repr__",
           [](const DenseTrafficLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.DenseTrafficLabelFunction";
           })
      .def(py::pickle(
          [](const DenseTrafficLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetRadius(),
                                  b.GetNumAgents());
          },
          [](py::tuple t) {
            if (t.size() != 3)
              throw std::runtime_error("Invalid label evaluator state!");
            return new DenseTrafficLabelFunction(t[0].cast<std::string>(),
                                                 t[1].cast<double>(),
                                                 t[2].cast<int>());
          }));

  py::class_<AgentNearLabelFunction, BaseLabelFunction,
             std::shared_ptr<AgentNearLabelFunction>>(m,
                                                      "AgentNearLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const AgentNearLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.AgentNearLabelFunction";
           })
      .def(py::pickle(
          [](const AgentNearLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetDistanceThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new AgentNearLabelFunction(t[0].cast<std::string>(),
                                              t[1].cast<double>());
          }));

  py::class_<EgoAccelerateLabelFunction, BaseLabelFunction,
             std::shared_ptr<EgoAccelerateLabelFunction>>(
      m, "EgoAccelerateLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const EgoAccelerateLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.EgoAccelerateLabelFunction";
           })
      .def(py::pickle(
          [](const EgoAccelerateLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetAccThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoAccelerateLabelFunction(t[0].cast<std::string>(),
                                                  t[1].cast<double>());
          }));

  py::class_<EgoRightmostLaneLabelFunction, BaseLabelFunction,
             std::shared_ptr<EgoRightmostLaneLabelFunction>>(
      m, "EgoRightmostLaneLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const EgoRightmostLaneLabelFunction& g) {
             return "bark.core.world.evaluation.ltl."
                    "EgoRightmostLaneLabelFunction";
           })
      .def(py::pickle(
          [](const EgoRightmostLaneLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetDistanceThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoRightmostLaneLabelFunction(t[0].cast<std::string>(),
                                                     t[1].cast<double>());
          }));

  py::class_<RightmostLaneLabelFunction, BaseLabelFunction,
             std::shared_ptr<RightmostLaneLabelFunction>>(
      m, "RightmostLaneLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const RightmostLaneLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.RightmostLaneLabelFunction";
           })
      .def(py::pickle(
          [](const RightmostLaneLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetDistanceThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new RightmostLaneLabelFunction(t[0].cast<std::string>(),
                                                  t[1].cast<double>());
          }));

  py::class_<EgoLeftmostLaneLabelFunction, BaseLabelFunction,
             std::shared_ptr<EgoLeftmostLaneLabelFunction>>(
      m, "EgoLeftmostLaneLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const EgoLeftmostLaneLabelFunction& g) {
             return "bark.core.world.evaluation.ltl."
                    "EgoLeftmostLaneLabelFunction";
           })
      .def(py::pickle(
          [](const EgoLeftmostLaneLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetDistanceThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoLeftmostLaneLabelFunction(t[0].cast<std::string>(),
                                                    t[1].cast<double>());
          }));

  py::class_<OnRoadLabelFunction, BaseLabelFunction,
             std::shared_ptr<OnRoadLabelFunction>>(m, "OnRoadLabelFunction")
      .def(py::init<const std::string&>())
      .def("__repr__",
           [](const OnRoadLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.OnRoadLabelFunction";
           })
      .def(py::pickle(
          [](const OnRoadLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new OnRoadLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<RelSpeedLabelFunction, BaseLabelFunction,
             std::shared_ptr<RelSpeedLabelFunction>>(m, "RelSpeedLabelFunction")
      .def(py::init<const std::string&, double>())
      .def("__repr__",
           [](const RelSpeedLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.RelSpeedLabelFunction";
           })
      .def(py::pickle(
          [](const RelSpeedLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetRelSpeedThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new RelSpeedLabelFunction(t[0].cast<std::string>(),
                                             t[1].cast<double>());
          }));

  py::class_<AgentAtLaneEndLabelFunction, BaseLabelFunction,
             std::shared_ptr<AgentAtLaneEndLabelFunction>>(
      m, "AgentAtLaneEndLabelFunction")
      .def(py::init<const std::string&, double>())
      .def(
          "__repr__",
          [](const AgentAtLaneEndLabelFunction& g) {
            return "bark.core.world.evaluation.ltl.AgentAtLaneEndLabelFunction";
          })
      .def(py::pickle(
          [](const AgentAtLaneEndLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetDistanceThres());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new AgentAtLaneEndLabelFunction(t[0].cast<std::string>(),
                                                   t[1].cast<double>());
          }));

  py::class_<RightOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<RightOfLabelFunction>>(m, "RightOfLabelFunction")
      .def(py::init<const std::string&>())
      .def("__repr__",
           [](const RightOfLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.RightOfLabelFunction";
           })
      .def(py::pickle(
          [](const RightOfLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new RightOfLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<LeftOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<LeftOfLabelFunction>>(m, "LeftOfLabelFunction")
      .def(py::init<const std::string&>())
      .def("__repr__",
           [](const LeftOfLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.LeftOfLabelFunction";
           })
      .def(py::pickle(
          [](const LeftOfLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new LeftOfLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<BehindOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<BehindOfLabelFunction>>(m, "BehindOfLabelFunction")
      .def(py::init<const std::string&>())
      .def("__repr__",
           [](const BehindOfLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.BehindOfLabelFunction";
           })
      .def(py::pickle(
          [](const BehindOfLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new BehindOfLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<FrontOfLabelFunction, BaseLabelFunction,
             std::shared_ptr<FrontOfLabelFunction>>(m, "FrontOfLabelFunction")
      .def(py::init<const std::string&>())
      .def("__repr__",
           [](const FrontOfLabelFunction& g) {
             return "bark.core.world.evaluation.ltl.FrontOfLabelFunction";
           })
      .def(py::pickle(
          [](const FrontOfLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new FrontOfLabelFunction(t[0].cast<std::string>());
          }));

  py::class_<AgentBeyondPointLabelFunction, BaseLabelFunction,
             std::shared_ptr<AgentBeyondPointLabelFunction>>(
      m, "AgentBeyondPointLabelFunction")
      .def(py::init<const std::string&, const Point2d&>())
      .def("__repr__",
           [](const AgentBeyondPointLabelFunction& g) {
             return "bark.core.world.evaluation.ltl."
                    "AgentBeyondPointLabelFunction";
           })
      .def(py::pickle(
          [](const AgentBeyondPointLabelFunction& b) {
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
      .def(py::init<const std::string&, const Point2d&>())
      .def(
          "__repr__",
          [](const EgoBeyondPointLabelFunction& g) {
            return "bark.core.world.evaluation.ltl.EgoBeyondPointLabelFunction";
          })
      .def(py::pickle(
          [](const EgoBeyondPointLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(), b.GetBeyondPoint());
          },
          [](py::tuple t) {
            if (t.size() != 2)
              throw std::runtime_error("Invalid label evaluator state!");
            return new EgoBeyondPointLabelFunction(t[0].cast<std::string>(),
                                                   t[1].cast<Point2d>());
          }));

  py::class_<SucceedingAgentLabelFunction, BaseLabelFunction,
             std::shared_ptr<SucceedingAgentLabelFunction>>(
      m, "SucceedingAgentLabelFunction")
      .def(py::init<const std::string&, bool, double>())
      .def(py::pickle(
          [](const SucceedingAgentLabelFunction& b) {
            return py::make_tuple(b.GetLabelStr(),
                                  b.GetUseFracLateralOffsetParam(),
                                  b.GetLateralDifferenceThreshold());
          },
          [](py::tuple t) {
            if (t.size() != 3)
              throw std::runtime_error("Invalid label evaluator state!");
            return new SucceedingAgentLabelFunction(t[0].cast<std::string>(),
                                                   t[2].cast<bool>(),
                                                   t[1].cast<double>());
          }));

  py::class_<
      GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>, BaseLabelFunction,
      std::shared_ptr<GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>>>(
      m, "CollisionEgoLabelFunction")
      .def(py::init<const std::string&>())
      .def(py::pickle(
          [](const GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>& b) {
            return py::make_tuple(b.GetLabelStr());
          },
          [](py::tuple t) {
            if (t.size() != 1)
              throw std::runtime_error("Invalid label evaluator state!");
            return new GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>(
                t[0].cast<std::string>());
          }));
}