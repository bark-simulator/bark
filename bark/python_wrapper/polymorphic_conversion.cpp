// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <typeinfo>
#include <string>
#include <memory>
#include <stdexcept>

#include "polymorphic_conversion.hpp"

#include "bark/commons/params/setter_params.hpp"
#include "bark/models/behavior/constant_velocity/constant_velocity.hpp"
#include "bark/models/behavior/dynamic_model/dynamic_model.hpp"
#include "bark/models/behavior/idm/idm_classic.hpp"
#include "bark/models/behavior/idm/idm_lane_tracking.hpp"
#include "bark/models/behavior/motion_primitives/macro_actions.hpp"
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
#include "bark/world/evaluation/evaluator_collision_ego_agent.hpp"
#include "bark/world/goal_definition/goal_definition_polygon.hpp"
#include "bark/world/goal_definition/goal_definition_sequential.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits.hpp"
#include "bark/world/goal_definition/goal_definition_state_limits_frenet.hpp"

#ifdef LTL_RULES
#include "bark/world/evaluation/ltl/labels/agent_beyond_point_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/ego_beyond_point_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/generic_ego_label_function.hpp"
#include "bark/world/evaluation/ltl/labels/preceding_agent_label_function.hpp"
#endif

#ifdef PLANNER_UCT
#include "src/behavior_uct_single_agent_macro_actions.hpp"
using modules::models::behavior::BehaviorUCTSingleAgentMacroActions;
#endif

#ifdef PLANNER_MVMCTS
#include "src/behavior_mcts_multi_agent.hpp"
using modules::models::behavior::BehaviorEGreedyMultiAgent;
using modules::models::behavior::BehaviorUCTMultiAgent;
#endif

namespace py = pybind11;

using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::world::goal_definition::GoalDefinitionStateLimitsFrenet;
using modules::world::goal_definition::GoalDefinitionSequential;
using modules::models::behavior::BehaviorIDMClassic;
using modules::models::behavior::BehaviorIDMLaneTracking;
using modules::models::behavior::BehaviorConstantVelocity;
using modules::models::behavior::BehaviorDynamicModel;
using modules::models::behavior::BehaviorStaticTrajectory;
using modules::models::behavior::BehaviorIntersectionRuleBased;
using modules::models::behavior::BehaviorLaneChangeRuleBased;
using modules::models::behavior::BehaviorMobilRuleBased;
using modules::models::behavior::BehaviorMobil;
using modules::models::behavior::primitives::PrimitiveConstAccStayLane;
using modules::commons::SetterParams;
using modules::models::behavior::primitives::Primitive;
using modules::models::behavior::primitives::PrimitiveConstAccChangeToLeft;
using modules::models::behavior::primitives::PrimitiveConstAccChangeToRight;
using modules::models::behavior::primitives::PrimitiveConstAccStayLane;

#ifdef LTL_RULES
using modules::world::evaluation::AgentBeyondPointLabelFunction;
using modules::world::evaluation::EgoBeyondPointLabelFunction;
using modules::world::evaluation::GenericEgoLabelFunction;
using modules::world::evaluation::PrecedingAgentLabelFunction;
#endif

py::tuple BehaviorModelToPython(BehaviorModelPtr behavior_model) {
  std::string behavior_model_name;
  if (!behavior_model) {
    behavior_model_name = "None";
    return py::make_tuple(behavior_model_name, behavior_model_name);
  }
  if (typeid(*behavior_model) == typeid(BehaviorConstantVelocity)) {
    behavior_model_name = "BehaviorConstantVelocity";
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
  } else if (typeid(*behavior_model) == typeid(BehaviorMPMacroActions)) {
    behavior_model_name = "BehaviorMPMacroActions";
  }
#ifdef PLANNER_UCT
  else if(typeid(*behavior_model) == typeid(BehaviorUCTSingleAgentMacroActions)) {
    behavior_model_name = "BehaviorUCTSingleAgentMacroActions";
  }
#endif
#ifdef PLANNER_MVMCTS
  else if (typeid(*behavior_model) == typeid(BehaviorUCTMultiAgent)) {
    behavior_model_name = "BehaviorUCTMultiAgent";
  } else if (typeid(*behavior_model) == typeid(BehaviorEGreedyMultiAgent)) {
    behavior_model_name = "BehaviorEGreedyMultiAgent";
  }
#endif
  else {
    LOG(ERROR) << "Unknown BehaviorType for polymorphic conversion.";
    throw;
  }
  return py::make_tuple(behavior_model, behavior_model_name);
}

BehaviorModelPtr PythonToBehaviorModel(py::tuple t) {
  std::string behavior_model_name = t[1].cast<std::string>();
  if (behavior_model_name.compare("BehaviorConstantVelocity") == 0) {
      return std::make_shared<BehaviorConstantVelocity>(
        t[0].cast<BehaviorConstantVelocity>());
  } else if (behavior_model_name.compare("BehaviorIDMLaneTracking") == 0) {
    return std::make_shared<BehaviorIDMLaneTracking>(
      t[0].cast<BehaviorIDMLaneTracking>());
  } else if (behavior_model_name.compare("BehaviorIDMClassic") == 0) {
    return std::make_shared<BehaviorIDMClassic>(
      t[0].cast<BehaviorIDMClassic>());
  } else if (behavior_model_name.compare("BehaviorIntersectionRuleBased") == 0) {
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
    return std::make_shared<BehaviorMobil>(
      t[0].cast<BehaviorMobil>());
  } else if (behavior_model_name.compare("PyBehaviorModel") == 0) {
    return std::make_shared<PyBehaviorModel>(
      t[0].cast<PyBehaviorModel>());
  } else if (behavior_model_name.compare("BehaviorDynamicModel") == 0) {
    return std::make_shared<BehaviorDynamicModel>(
      t[0].cast<BehaviorDynamicModel>());
  } else if (behavior_model_name.compare("BehaviorMPMacroActions") == 0) {
    return std::make_shared<BehaviorMPMacroActions>(
        t[0].cast<BehaviorMPMacroActions>());
  }
#ifdef PLANNER_UCT
  else if(behavior_model_name.compare("BehaviorUCTSingleAgentMacroActions") == 0) {
    return std::make_shared<BehaviorUCTSingleAgentMacroActions>(
      t[0].cast<BehaviorUCTSingleAgentMacroActions>());

  }
#endif
#ifdef PLANNER_MVMCTS
  else if (behavior_model_name.compare("BehaviorUCTMultiAgent") == 0) {
    return std::make_shared<BehaviorUCTMultiAgent>(
        t[0].cast<BehaviorUCTMultiAgent>());
  } else if (behavior_model_name.compare("BehaviorEGreedyMultiAgent") == 0) {
    return std::make_shared<BehaviorEGreedyMultiAgent>(
        t[0].cast<BehaviorEGreedyMultiAgent>());
  }
#endif
  else if (behavior_model_name.compare("None") == 0) {
    return nullptr;
  } else {
    LOG(ERROR) << "Unknown BehaviorType " << behavior_model_name
               << " for polymorphic conversion.";
    throw;
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
  } else if (typeid(*goal_definition) == typeid(GoalDefinitionStateLimitsFrenet)) {
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
  } else if (goal_definition_name.compare("GoalDefinitionStateLimitsFrenet") == 0) {
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
  const auto param_list = t[0].cast<modules::commons::CondensedParamList>();
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

#ifdef LTL_RULES
py::tuple LabelToPython(const LabelFunctionPtr& label) {
  std::string label_name;
  if (typeid(*label) == typeid(AgentBeyondPointLabelFunction)) {
    label_name = "AgentBeyondPointLabelFunction";
    return py::make_tuple(label, label_name);
  } else if (typeid(*label) == typeid(EgoBeyondPointLabelFunction)) {
    label_name = "EgoBeyondPointLabelFunction";
    return py::make_tuple(label, label_name);
  } else if (typeid(*label) == typeid(PrecedingAgentLabelFunction)) {
    label_name = "PrecedingAgentLabelFunction";
    return py::make_tuple(label, label_name);
  } else if (typeid(*label) ==
             typeid(GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>)) {
    label_name = "CollisionEgoLabelFunction";
    return py::make_tuple(label, label_name);
  } else {
    LOG(ERROR) << "Unknown LabelType for polymorphic conversion.";
    throw;
  }
  // Should never be reached
  return py::make_tuple(label, label_name);
}
LabelFunctionPtr PythonToLabel(py::tuple t) {
  std::string label_name = t[1].cast<std::string>();
  if (label_name.compare("AgentBeyondPointLabelFunction") == 0) {
    return std::make_shared<AgentBeyondPointLabelFunction>(
        t[0].cast<AgentBeyondPointLabelFunction>());
  } else if (label_name.compare("EgoBeyondPointLabelFunction") == 0) {
    return std::make_shared<EgoBeyondPointLabelFunction>(
        t[0].cast<EgoBeyondPointLabelFunction>());
  } else if (label_name.compare("PrecedingAgentLabelFunction") == 0) {
    return std::make_shared<PrecedingAgentLabelFunction>(
        t[0].cast<PrecedingAgentLabelFunction>());
  } else if (label_name.compare("CollisionEgoLabelFunction") == 0) {
    return std::make_shared<
        GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>>(
        t[0].cast<GenericEgoLabelFunction<EvaluatorCollisionEgoAgent>>());
  } else {
    LOG(ERROR) << "Unknown LabelType for polymorphic conversion.";
    throw;
  }
  return nullptr;
}
#endif