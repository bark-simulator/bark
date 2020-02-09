// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_POLYMORPHIC_CONVERSION_HPP_
#define PYTHON_POLYMORPHIC_CONVERSION_HPP_

#include "python/common.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"

namespace py = pybind11;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::models::behavior::BehaviorModelPtr;
using modules::commons::ParamsPtr;

// For pickle we need conversion functions between the genereric base types and the derived types

// Behavior Models
py::tuple BehaviorModelToPython(BehaviorModelPtr behavior_model);
BehaviorModelPtr PythonToBehaviorModel(py::tuple t);

// Goal Definition
py::tuple GoalDefinitionToPython(GoalDefinitionPtr goal_definition);
GoalDefinitionPtr PythonToGoalDefinition(py::tuple t);

py::tuple ParamsToPython(const ParamsPtr& params);
ParamsPtr PythonToParams(py::tuple t);

// Todo: Other polymorphic types, e.g. execution model...

#endif  // PYTHON_POLYMORPHIC_CONVERSION_HPP_
