// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_WORLD_EVALUATION_HPP_
#define PYTHON_PYTHON_BINDINGS_WORLD_EVALUATION_HPP_

#include "bark/world/evaluation/labels/base_label_function.hpp"
#include "bark/python_wrapper/common.hpp"
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

class PyBaseLabelFunction : public BaseLabelFunction {
 public:
  using BaseLabelFunction::BaseLabelFunction;

  std::vector<LabelMap::value_type> Evaluate(
      const ObservedWorld &observed_world) const override {
    PYBIND11_OVERLOAD_PURE(
        std::vector<LabelMap::value_type>, BaseLabelFunction,
                           Evaluate,
        observed_world
    );
  }
};

void python_evaluation(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_WORLD_EVALUATION_HPP_