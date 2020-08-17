// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/python_wrapper/models/models.hpp"
#include "bark/python_wrapper/models/behavior.hpp"
#include "bark/python_wrapper/models/dynamic.hpp"
#include "bark/python_wrapper/models/execution.hpp"

void python_models(py::module m) {
  python_behavior(m.def_submodule("behavior", "Behavior wrapping"));
  python_execution(m.def_submodule(
      "execution", "submodule containing all wrapped execution models"));
  python_dynamic(m.def_submodule(
      "dynamic", "submodule containing all wrapped dynamic models"));
}
