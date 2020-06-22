// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_
#define BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_

#include "bark/world/evaluation/ltl/labels/base_label_function.hpp"
#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;
using namespace modules::world::evaluation;
using namespace modules::world;


class PyBaseLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;

  LabelMap Evaluate(const ObservedWorld& observed_world) const override {
    PYBIND11_OVERLOAD_PURE(LabelMap, BaseLabelFunction, Evaluate,
                           observed_world
    );
  }
};

void python_ltl(py::module m);

#endif  // BARK_PYTHON_WRAPPER_WORLD_LTL_HPP_
