// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <typeinfo>
#include <string>
#include <memory>
#include <stdexcept>

#include "polymorphic_conversion.hpp"

#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/idm/idm_classic.hpp"
#include "modules/models/behavior/mobil/mobil.hpp"
#include "modules/models/behavior/static_trajectory/behavior_static_trajectory.hpp"
#include "modules/world/goal_definition/goal_definition_polygon.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits.hpp"
#include "modules/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include "modules/world/goal_definition/goal_definition_sequential.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "python/models/behavior.hpp"

#ifdef PLANNER_UCT
#include "src/behavior_uct_single_agent_macro_actions.hpp"
using modules::models::behavior::BehaviorUCTSingleAgentMacroActions;
#endif




namespace py = pybind11;

using modules::world::goal_definition::GoalDefinitionPolygon;
using modules::world::goal_definition::GoalDefinitionStateLimits;
using modules::world::goal_definition::GoalDefinitionStateLimitsFrenet;
using modules::world::goal_definition::GoalDefinitionSequential;
using modules::models::behavior::BehaviorIDMClassic;
using modules::models::behavior::BehaviorConstantVelocity;
using modules::models::behavior::BehaviorStaticTrajectory;
using modules::models::behavior::BehaviorMobil;
using modules::commons::SetterParams;

py::tuple BehaviorModelToPython(BehaviorModelPtr behavior_model) {
  std::string behavior_model_name;
  if (typeid(*behavior_model) == typeid(BehaviorConstantVelocity)) {
    behavior_model_name = "BehaviorConstantVelocity";
  } else if (typeid(*behavior_model) == typeid(BehaviorIDMClassic)) {
    behavior_model_name = "BehaviorIDMClassic";
  } else if (typeid(*behavior_model) == typeid(BehaviorStaticTrajectory)) {
    behavior_model_name = "BehaviorStaticTrajectory";
  } else if (typeid(*behavior_model) == typeid(BehaviorMobil)) {
    behavior_model_name = "BehaviorMobil";
  } else if (typeid(*behavior_model) == typeid(PyBehaviorModel)) {
    behavior_model_name = "PyBehaviorModel";
  }
#ifdef PLANNER_UCT
  else if(typeid(*behavior_model) == typeid(BehaviorUCTSingleAgentMacroActions)) {
    behavior_model_name = "BehaviorUCTSingleAgentMacroActions";
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
  } else if (behavior_model_name.compare("BehaviorIDMClassic") == 0) {
    return std::make_shared<BehaviorIDMClassic>(
      t[0].cast<BehaviorIDMClassic>());
  } else if (behavior_model_name.compare("BehaviorStaticTrajectory") == 0) {
    return std::make_shared<BehaviorStaticTrajectory>(
      t[0].cast<BehaviorStaticTrajectory>());
  } else if (behavior_model_name.compare("BehaviorMobil") == 0) {
    return std::make_shared<BehaviorMobil>(
      t[0].cast<BehaviorMobil>());
  } else if (behavior_model_name.compare("PyBehaviorModel") == 0) {
    return std::make_shared<PyBehaviorModel>(
      t[0].cast<PyBehaviorModel>());
  }
#ifdef PLANNER_UCT
  else if(behavior_model_name.compare("BehaviorUCTSingleAgentMacroActions") == 0) {
    return std::make_shared<BehaviorUCTSingleAgentMacroActions>(
      t[0].cast<BehaviorUCTSingleAgentMacroActions>());

  }
#endif
  else {
    LOG(ERROR) << "Unknown BehaviorType for polymorphic conversion.";
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
