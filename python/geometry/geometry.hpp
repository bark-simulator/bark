// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_GEOMETRY_GEOMETRY_HPP_
#define PYTHON_PYTHON_BINDINGS_GEOMETRY_GEOMETRY_HPP_

#include "python/common.hpp"
#include "modules/commons/params/params.hpp"
#include "modules/commons/params/params_test.h"

namespace py = pybind11;


void python_geometry(py::module m);


#endif  // PYTHON_PYTHON_BINDINGS_GEOMETRY_GEOMETRY_HPP_
