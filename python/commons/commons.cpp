// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "commons.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/runtime/tests/py_param_server_test_helper.hpp"

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
      .def("setInt", &Params::SetInt);
      /*.def(py::pickle(
      [](const Params* p) -> py::tuple {
        return py::make_tuple(dynamic_cast<const PyParams*>(p)->GetCondensedParamList());
      },
      [](py::tuple  &t)  {
        CondensedParamList param_list =  t[0].cast<CondensedParamList>();
        return new SetterParams(false, param_list);
      }));*/


    m.def("ParamsTest", &DoSomeParams);

    py::class_<CppParamServerTestObject, std::shared_ptr<CppParamServerTestObject>>(m, "CppParamServerTestObject")
      .def(py::init<std::shared_ptr<modules::commons::Params>>())
      .def("GetRealValue", &CppParamServerTestObject::GetRealValue)
      .def("GetBoolValueTrue", &CppParamServerTestObject::GetBoolValueTrue)
      .def("GetBoolValueFalse", &CppParamServerTestObject::GetBoolValueFalse)
      .def("GetIntValue", &CppParamServerTestObject::GetIntValue)
      .def("GetListListFloatValue", &CppParamServerTestObject::GetListListFloatValue)
      .def("GetParams", &CppParamServerTestObject::GetParams)
      .def(py::pickle(
      [](const CppParamServerTestObject& p) -> py::tuple {
        return py::make_tuple(p.GetParams()->GetCondensedParamList());
      },
      [](py::tuple  &t)  {
          const auto param_list = t[0].cast<CondensedParamList>();
          return CppParamServerTestObject(std::make_shared<SetterParams>(true, param_list));
      }));

}

}  // namespace commons
}  // namespace modules
