// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/python_wrapper/world/prediction.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include "bark/world/prediction/prediction_settings.hpp"

namespace py = pybind11;
using bark::models::behavior::BehaviorModelPtr;
using bark::world::objects::AgentId;
using bark::world::prediction::PredictionSettings;

void python_prediction(py::module m) {
  py::class_<PredictionSettings, std::shared_ptr<PredictionSettings>>(
      m, "PredictionSettings")
      .def(py::init<const BehaviorModelPtr&, const BehaviorModelPtr&,
                    const BehaviorModelPtr&, const std::vector<AgentId>&>())
      .def(py::pickle(
          [](const PredictionSettings& b) {
            return py::make_tuple(
                BehaviorModelToPython(b.ego_prediction_model_),
                BehaviorModelToPython(b.default_prediction_model_),
                BehaviorModelToPython(b.specific_prediction_model_),
                b.specific_prediction_agents_);
          },
          [](py::tuple t) {
            if (t.size() != 4)
              throw std::runtime_error("Invalid prediction settings state!");
            auto specific_agents_set = t[3].cast<std::set<AgentId>>();
            std::vector<AgentId> specific_agents_v(specific_agents_set.begin(),
                                                   specific_agents_set.end());
            return new PredictionSettings(
                PythonToBehaviorModel(t[0].cast<py::tuple>()),
                PythonToBehaviorModel(t[1].cast<py::tuple>()),
                PythonToBehaviorModel(t[2].cast<py::tuple>()),
                specific_agents_v);
          }));
}
