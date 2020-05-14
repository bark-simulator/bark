// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_WORLD_EVALUATION_HPP_
#define PYTHON_PYTHON_BINDINGS_WORLD_EVALUATION_HPP_

#include "bark/pybark/common.hpp"
#include "bark/world/evaluation/base_evaluator.hpp"

namespace py = pybind11;
using namespace modules::world::evaluation;
using namespace modules::world;

class PyBaseEvaluator : public BaseEvaluator {
 public:
  using BaseEvaluator::BaseEvaluator;

  EvaluationReturn Evaluate(const World& world) override {
    PYBIND11_OVERLOAD_PURE(
        EvaluationReturn,
        BaseEvaluator,
        Evaluate,
        world);
  }
};

void python_evaluation(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_WORLD_EVALUATION_HPP_