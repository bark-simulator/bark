// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_MODELS_DYNAMIC_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_DYNAMIC_HPP_

#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

using namespace bark::models::dynamic;

class PyDynamicModel : public DynamicModel {
 public:
  using DynamicModel::DynamicModel;

  State StateSpaceModel(const State& x, const Input& u) const override {
    PYBIND11_OVERLOAD_PURE(State, DynamicModel, StateSpaceModel, x, u);
  }

  std::shared_ptr<DynamicModel> Clone() const override {
    PYBIND11_OVERLOAD_PURE(std::shared_ptr<DynamicModel>, DynamicModel, clone);
  }
};

void python_dynamic(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_DYNAMIC_HPP_
