// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_MODELS_DYNAMIC_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_DYNAMIC_HPP_

#include "python/common.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace py = pybind11;

using namespace modules::models::dynamic;

class PyDynamicModel : public DynamicModel {
 public:
  using DynamicModel::DynamicModel;

  State StateSpaceModel(const State &x, const Input &u) const override {
    PYBIND11_OVERLOAD_PURE(
        State,
        DynamicModel,
        StateSpaceModel,
        x, u);
  }

  std::shared_ptr<DynamicModel> Clone() const override {
    PYBIND11_OVERLOAD_PURE(
        std::shared_ptr<DynamicModel>,
        DynamicModel,
        clone);
  }
};

void python_dynamic(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_DYNAMIC_HPP_
