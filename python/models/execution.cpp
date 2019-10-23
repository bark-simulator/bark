// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "execution.hpp"
#include "modules/models/execution/interpolation/interpolate.hpp"
//#include "modules/models/execution/mpc/mpc.hpp"

namespace py = pybind11;
using namespace modules::models::dynamic;
using namespace modules::models::execution;
using namespace modules::commons;
using std::shared_ptr;

void python_execution(py::module m) {
  py::class_<ExecutionModel,
             PyExecutionModel,
             ExecutionModelPtr>(m, "ExecutionModel")
      .def(py::init<Params *>())
      .def("execute", &ExecutionModel::Execute)
      .def_property_readonly("last_trajectory",
                             &ExecutionModel::get_last_trajectory);

  py::class_<ExecutionModelInterpolate,
             ExecutionModel,
             shared_ptr<ExecutionModelInterpolate>>(m,
                                                    "ExecutionModelInterpolate")
      .def(py::init<Params *>())
      .def("__repr__", [](const ExecutionModelInterpolate &m) {
        return "bark.dynamic.ExecutionModelInterpolate";
      })
      .def(py::pickle(
        [](const ExecutionModelInterpolate &m) -> std::string { 
            return "ExecutionModelInterpolate"; // 0
        },
        [](std::string s) { // __setstate__
            if (s != "ExecutionModelInterpolate" )
                throw std::runtime_error("Invalid tyoe of execution model!");

            /* Create a new C++ instance */
            return new ExecutionModelInterpolate(nullptr); // param pointer must be set via python
        }));
     

  // py::class_<ExecutionModelMpc,
  //            ExecutionModel,
  //            shared_ptr<ExecutionModelMpc>>(m, "ExecutionModelMpc")
  //     .def(py::init<Params *>())
  //     .def("__repr__", [](const ExecutionModelMpc &m) {
  //       return "bark.dynamic.ExecutionModelMpc";
  //     })
  //     .def_property_readonly("last_weights",
  //                            &ExecutionModelMpc::get_last_weights)
  //     .def_property_readonly("last_desired_states_",
  //                            &ExecutionModelMpc::get_last_desired_states);
}
