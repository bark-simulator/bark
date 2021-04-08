// Copyright (c) 2021 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_WORLD_RENDERER_HPP_
#define PYTHON_WORLD_RENDERER_HPP_

#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

void python_renderer(py::module m);

#endif  // PYTHON_WORLD_RENDERER_HPP_
