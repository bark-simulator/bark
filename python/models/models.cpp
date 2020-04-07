// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "python/models/models.hpp"
#include "python/models/dynamic.hpp"
#include "python/models/behavior.hpp"
#include "python/models/execution.hpp"


void python_models(py::module m) {
  python_behavior(
    m.def_submodule("behavior", "Behavior wrapping"));
  python_execution(
    m.def_submodule("execution",
    "submodule containing all wrapped execution models"));
  python_dynamic(
    m.def_submodule("dynamic",
    "submodule containing all wrapped dynamic models"));
}
