// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "commons.hpp"
#include "modules/commons/params/setter_params.hpp"

namespace py = pybind11;

namespace modules {
namespace commons {

void python_commons(py::module m) {
    py::class_<Params, PyParams, std::shared_ptr<Params>>(m, "Params")
      .def(py::init<>())
      .def("Access", &Params::operator[])
      .def("addChild", &Params::AddChild)
      .def("getBool", &Params::GetBool)
      .def("getReal", &Params::GetReal)
      .def("getInt", &Params::GetInt)
      .def("setBool", &Params::SetBool)
      .def("setReal", &Params::SetReal)
      .def("getCondensedParamList", &Params::GetCondensedParamList)
      .def("setInt", &Params::SetInt)
      .def(py::pickle(
      [](const Params* p) -> py::tuple {
        return py::make_tuple(p->GetCondensedParamList());
      },
      [](py::tuple  &t)  {
        CondensedParamList param_list =  t[0].cast<CondensedParamList>();
        return new SetterParams(false, param_list);
      }));
    m.def("ParamsTest", &DoSomeParams);
}

}  // namespace commons
}  // namespace modules
