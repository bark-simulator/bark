// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_GEOMETRY_GEOMETRY_HPP_
#define PYTHON_PYTHON_BINDINGS_GEOMETRY_GEOMETRY_HPP_

#include "bark/commons/params/params.hpp"
#include "bark/commons/params/params_test.h"
#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

void python_geometry(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_GEOMETRY_GEOMETRY_HPP_
