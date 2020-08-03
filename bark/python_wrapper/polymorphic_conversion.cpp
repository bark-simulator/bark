// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "polymorphic_conversion.hpp"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/constant_acceleration/constant_acceleration.hpp"
#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_left.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_change_to_right.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_const_acc_stay_lane.hpp"
#include "bark/models/behavior/motion_primitives/primitives/primitive_gap_keeping.hpp"
#include "bark/models/behavior/rule_based/intersection_behavior.hpp"
#include "bark/models/behavior/rule_based/lane_change_behavior.hpp"
#include "bark/models/behavior/rule_based/mobil.hpp"
#include "bark/models/behavior/rule_based/mobil_behavior.hpp"
#include "bark/models/behavior/static_trajectory/behavior_static_trajectory.hpp"
#include "bark/python_wrapper/models/behavior.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/goal_definition/goal_definition_sequential.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits_frenet.hpp"

#include "bark/models/behavior/idm/stochastic/idm_stochastic.hpp"

#ifdef PLANNER_UCT
#include "bark_mcts/models/behavior/behavior_uct_hypothesis.hpp"
#include "bark_mcts/models/behavior/hypothesis/idm/hypothesis_idm.hpp"
using bark::models::behavior::BehaviorHypothesisIDM;
using bark::models::behavior::BehaviorUCTHypothesis;
#endif

namespace py = pybind11;

using bark::commons::SetterParams;
using bark::models::behavior::BehaviorConstantAcceleration;
using bark::models::behavior::BehaviorDynamicModel;
using bark::models::behavior::BehaviorIDMClassic;
using bark::models::behavior::BehaviorIDMLaneTracking;
using bark::models::behavior::BehaviorIDMStochastic;
using bark::models::behavior::BehaviorIntersectionRuleBased;
using bark::models::behavior::BehaviorLaneChangeRuleBased;
using bark::models::behavior::BehaviorMobil;
using bark::models::behavior::BehaviorMobilRuleBased;
using bark::models::behavior::BehaviorStaticTrajectory;
using bark::models::behavior::primitives::Primitive;
using bark::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
using bark::models::behavior::primitives::PrimitiveConstAccChangeToRight;
using bark::models::behavior::primitives::PrimitiveConstAccStayLane;
using bark::models::behavior::primitives::PrimitiveGapKeeping;
using bark::world::goal_definition::GoalDefinitionPolygon;
using bark::world::goal_definition::GoalDefinitionSequential;
using bark::world::goal_definition::GoalDefinitionStateLimits;
using bark::world::goal_definition::GoalDefinitionStateLimitsFrenet;

py::tuple BehaviorModelToPython(BehaviorModelPtr behavior_model) {
  std::string behavior_model_name;
  if (typeid(*behavior_model) == typeid(BehaviorConstantAcceleration)) {
    behavior_model_name = "BehaviorConstantAcceleration";
  } else if (typeid(*behavior_model) == typeid(BehaviorIDMLaneTracking)) {
    behavior_model_name = "BehaviorIDMLaneTracking";
  } else if (typeid(*behavior_model) == typeid(BehaviorIDMClassic)) {
    behavior_model_name = "BehaviorIDMClassic";
  } else if (typeid(*behavior_model) == typeid(BehaviorIntersectionRuleBased)) {
    behavior_model_name = "BehaviorIntersectionRuleBased";
  } else if (typeid(*behavior_model) == typeid(BehaviorLaneChangeRuleBased)) {
    behavior_model_name = "BehaviorLaneChangeRuleBased";
  } else if (typeid(*behavior_model) == typeid(BehaviorMobilRuleBased)) {
    behavior_model_name = "BehaviorMobilRuleBased";
  } else if (typeid(*behavior_model) == typeid(BehaviorStaticTrajectory)) {
    behavior_model_name = "BehaviorStaticTrajectory";
  } else if (typeid(*behavior_model) == typeid(BehaviorMobil)) {
    behavior_model_name = "BehaviorMobil";
  } else if (typeid(*behavior_model) == typeid(BehaviorDynamicModel)) {
    behavior_model_name = "BehaviorDynamicModel";
  } else if (typeid(*behavior_model) == typeid(PyBehaviorModel)) {
    behavior_model_name = "PyBehaviorModel";
  } else if (typeid(*behavior_model) == typeid(BehaviorIDMStochastic)) {
    behavior_model_name = "BehaviorIDMStochastic";
  }
#ifdef PLANNER_UCT
  else if (typeid(*behavior_model) == typeid(BehaviorUCTHypothesis)) {
    behavior_model_name = "BehaviorUCTHypothesis";
  } else if (typeid(*behavior_model) == typeid(BehaviorHypothesisIDM)) {
    behavior_model_name = "BehaviorHypothesisIDM";
  }
#endif
  else {
    LOG(FATAL) << "Unknown BehaviorType for polymorphic conversion to python: "
               << typeid(*behavior_model).name();
  }
  return py::make_tuple(behavior_model, behavior_model_name);
}

BehaviorModelPtr PythonToBehaviorModel(py::tuple t) {
  std::string behavior_model_name = t[1].cast<std::string>();
  if (behavior_model_name.compare("BehaviorConstantAcceleration") == 0) {
    return std::make_shared<BehaviorConstantAcceleration>(
        t[0].cast<BehaviorConstantAcceleration>());
  } else if (behavior_model_name.compare("BehaviorIDMLaneTracking") == 0) {
    return std::make_shared<BehaviorIDMLaneTracking>(
        t[0].cast<BehaviorIDMLaneTracking>());
  } else if (behavior_model_name.compare("BehaviorIDMClassic") == 0) {
    return std::make_shared<BehaviorIDMClassic>(
        t[0].cast<BehaviorIDMClassic>());
  } else if (behavior_model_name.compare("BehaviorIntersectionRuleBased") ==
             0) {
    return std::make_shared<BehaviorIntersectionRuleBased>(
        t[0].cast<BehaviorIntersectionRuleBased>());
  } else if (behavior_model_name.compare("BehaviorLaneChangeRuleBased") == 0) {
    return std::make_shared<BehaviorLaneChangeRuleBased>(
        t[0].cast<BehaviorLaneChangeRuleBased>());
  } else if (behavior_model_name.compare("BehaviorStaticTrajectory") == 0) {
    return std::make_shared<BehaviorStaticTrajectory>(
        t[0].cast<BehaviorStaticTrajectory>());
  } else if (behavior_model_name.compare("BehaviorMobilRuleBased") == 0) {
    return std::make_shared<BehaviorMobilRuleBased>(
        t[0].cast<BehaviorMobilRuleBased>());
  } else if (behavior_model_name.compare("BehaviorMobil") == 0) {
    return std::make_shared<BehaviorMobil>(t[0].cast<BehaviorMobil>());
  } else if (behavior_model_name.compare("PyBehaviorModel") == 0) {
    return std::make_shared<PyBehaviorModel>(t[0].cast<PyBehaviorModel>());
  } else if (behavior_model_name.compare("BehaviorDynamicModel") == 0) {
    return std::make_shared<BehaviorDynamicModel>(
        t[0].cast<BehaviorDynamicModel>());
  } else if (behavior_model_name.compare("BehaviorIDMStochastic") == 0) {
    return std::make_shared<BehaviorIDMStochastic>(
        t[0].cast<BehaviorIDMStochastic>());
  }
#ifdef PLANNER_UCT
  else if (behavior_model_name.compare("BehaviorUCTHypothesis") == 0) {
    return std::make_shared<BehaviorUCTHypothesis>(
        t[0].cast<BehaviorUCTHypothesis>());
  } else if (behavior_model_name.compare("BehaviorHypothesisIDM") == 0) {
    return std::make_shared<BehaviorHypothesisIDM>(
        t[0].cast<BehaviorHypothesisIDM>());
  }
#endif
  else {
    LOG(FATAL) << "Unknown BehaviorType for polymorphic conversion to C++ : "
               << behavior_model_name;
  }
}

py::tuple GoalDefinitionToPython(GoalDefinitionPtr goal_definition) {
  std::string goal_definition_name;
  if (typeid(*goal_definition) == typeid(GoalDefinitionPolygon)) {
    goal_definition_name = "GoalDefinitionPolygon";
  } else if (typeid(*goal_definition) == typeid(GoalDefinitionStateLimits)) {
    goal_definition_name = "GoalDefinitionStateLimits";
  } else if (typeid(*goal_definition) == typeid(GoalDefinitionSequential)) {
    goal_definition_name = "GoalDefinitionSequential";
  } else if (typeid(*goal_definition) ==
             typeid(GoalDefinitionStateLimitsFrenet)) {
    goal_definition_name = "GoalDefinitionStateLimitsFrenet";
  } else {
    LOG(ERROR) << "Unknown GoalDefinitionType for polymorphic conversion.";
    throw;
  }
  return py::make_tuple(goal_definition, goal_definition_name);
}

GoalDefinitionPtr PythonToGoalDefinition(py::tuple t) {
  std::string goal_definition_name = t[1].cast<std::string>();
  if (goal_definition_name.compare("GoalDefinitionPolygon") == 0) {
    return std::make_shared<GoalDefinitionPolygon>(
        t[0].cast<GoalDefinitionPolygon>());
  } else if (goal_definition_name.compare("GoalDefinitionStateLimits") == 0) {
    return std::make_shared<GoalDefinitionStateLimits>(
        t[0].cast<GoalDefinitionStateLimits>());
  } else if (goal_definition_name.compare("GoalDefinitionSequential") == 0) {
    return std::make_shared<GoalDefinitionSequential>(
        t[0].cast<GoalDefinitionSequential>());
  } else if (goal_definition_name.compare("GoalDefinitionStateLimitsFrenet") ==
             0) {
    return std::make_shared<GoalDefinitionStateLimitsFrenet>(
        t[0].cast<GoalDefinitionStateLimitsFrenet>());
  } else {
    LOG(ERROR) << "Unknown GoalDefinitionType for polymorphic conversion.";
    throw;
  }
}

py::tuple ParamsToPython(const ParamsPtr& params) {
  return py::make_tuple(params->GetCondensedParamList());
}

ParamsPtr PythonToParams(py::tuple t) {
  const auto param_list = t[0].cast<bark::commons::CondensedParamList>();
  return std::make_shared<SetterParams>(true, param_list);
}

py::tuple PrimitiveToPython(const PrimitivePtr& prim) {
  std::string label_name;
  if (typeid(*prim) == typeid(PrimitiveGapKeeping)) {
    label_name = "PrimitiveGapKeeping";
  } else if (typeid(*prim) == typeid(PrimitiveConstAccStayLane)) {
    label_name = "PrimitiveConstAccStayLane";
  } else if (typeid(*prim) == typeid(PrimitiveConstAccChangeToLeft)) {
    label_name = "PrimitiveConstAccChangeToLeft";
  } else if (typeid(*prim) == typeid(PrimitiveConstAccChangeToRight)) {
    label_name = "PrimitiveConstAccChangeToRight";
  } else {
    LOG(ERROR) << "Unknown Primitive type for polymorphic conversion.";
    throw;
  }
  return py::make_tuple(prim, label_name);
}
PrimitivePtr PythonToPrimitive(py::tuple t) {
  std::string label_name = t[1].cast<std::string>();
  if (label_name.compare("PrimitiveGapKeeping") == 0) {
    return std::make_shared<PrimitiveGapKeeping>(
        t[0].cast<PrimitiveGapKeeping>());
  } else if (label_name.compare("PrimitiveConstAccStayLane") == 0) {
    return std::make_shared<PrimitiveConstAccStayLane>(
        t[0].cast<PrimitiveConstAccStayLane>());
  } else if (label_name.compare("PrimitiveConstAccChangeToLeft") == 0) {
    return std::make_shared<PrimitiveConstAccChangeToLeft>(
        t[0].cast<PrimitiveConstAccChangeToLeft>());
  } else if (label_name.compare("PrimitiveConstAccChangeToRight") == 0) {
    return std::make_shared<PrimitiveConstAccChangeToRight>(
        t[0].cast<PrimitiveConstAccChangeToRight>());
  } else {
    LOG(ERROR) << "Unknown LabelType for polymorphic conversion.";
    throw;
  }
  return nullptr;
}