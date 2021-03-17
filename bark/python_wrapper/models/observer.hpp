// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_MODELS_OBSERVER_HPP_
#define PYTHON_PYTHON_BINDINGS_MODELS_OBSERVER_HPP_
#include "bark/models/Observer/observer_model.hpp"
#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;
using namespace bark::models::observer;
using bark::world::AgentId;
using bark::world::dWorldPtr;

class PyObserverModel : public ObserverModel {
 public:
  using ObserverModel::ObserverModel;

  ObservedWorld Observe(
    const WorldPtr& world, const AgentId& agent_id) override {
    PYBIND11_OVERLOAD_PURE(
      ObservedWorld, ObserverModel, Observe, world, agent_id);
  }

  std::shared_ptr<ObserverModel> Clone() const override {
    PYBIND11_OVERLOAD_PURE(std::shared_ptr<ObserverModel>, ObserverModel,
                           clone);
  }
};

void python_observer(py::module m);

#endif  // PYTHON_PYTHON_BINDINGS_MODELS_OBSERVER_HPP_
