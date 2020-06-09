// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_MODELS_EXECUTION_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_EXECUTION_HPP_
#include "python/common.hpp"
#include "modules/models/execution/execution_model.hpp"

namespace py = pybind11;
using namespace modules::models::dynamic;
using namespace modules::models::execution;

class PyExecutionModel : public ExecutionModel {
 public:
  using ExecutionModel::ExecutionModel;

  void Execute(
    const float &delta_time,
    const Trajectory &trajectory,
    const DynamicModelPtr dynamic_model) override {
      PYBIND11_OVERLOAD_PURE(
        void,
        ExecutionModel,
        Execute,
        delta_time,
        trajectory,
        dynamic_model);
    }

    std::shared_ptr<ExecutionModel> Clone() const override {
    PYBIND11_OVERLOAD_PURE(
      std::shared_ptr<ExecutionModel>,
      ExecutionModel,
      clone);
  }
};

void python_execution(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_EXECUTION_HPP_
