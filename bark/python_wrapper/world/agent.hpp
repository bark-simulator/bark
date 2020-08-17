// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_WORLD_AGENT_HPP_
#define PYTHON_PYTHON_BINDINGS_WORLD_AGENT_HPP_

#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

void python_agent(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_WORLD_AGENT_HPP_
