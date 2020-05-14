// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_POLYMORPHIC_CONVERSION_HPP_
#define PYTHON_POLYMORPHIC_CONVERSION_HPP_

<<<<<<< HEAD:bark/core/polymorphic_conversion.hpp
<<<<<<< HEAD:python/polymorphic_conversion.hpp
#include "python/common.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/world/goal_definition/goal_definition.hpp"
#include "modules/models/behavior/motion_primitives/primitives/primitive.hpp"

=======
#include "bark/core/common.hpp"
=======
#include "bark/pybark/common.hpp"
>>>>>>> Wrapped C++ bark in pybark:bark/pybark/polymorphic_conversion.hpp
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/goal_definition/goal_definition.hpp"
>>>>>>> Package Restructuring:bark/core/polymorphic_conversion.hpp

namespace py = pybind11;
using modules::world::goal_definition::GoalDefinitionPtr;
using modules::models::behavior::BehaviorModelPtr;
using modules::commons::ParamsPtr;
using modules::models::behavior::primitives::PrimitivePtr;


// For pickle we need conversion functions between the genereric base types and the derived types

// Behavior Models
py::tuple BehaviorModelToPython(BehaviorModelPtr behavior_model);
BehaviorModelPtr PythonToBehaviorModel(py::tuple t);

// Goal Definition
py::tuple GoalDefinitionToPython(GoalDefinitionPtr goal_definition);
GoalDefinitionPtr PythonToGoalDefinition(py::tuple t);

py::tuple ParamsToPython(const ParamsPtr& params);
ParamsPtr PythonToParams(py::tuple t);

py::tuple PrimitiveToPython(const PrimitivePtr& prim);
PrimitivePtr PythonToPrimitive(py::tuple t);

// Todo: Other polymorphic types, e.g. execution model...

#endif  // PYTHON_POLYMORPHIC_CONVERSION_HPP_
