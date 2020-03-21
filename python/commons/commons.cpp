// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "commons.hpp"
#include "python/polymorphic_conversion.hpp"
#include "modules/commons/params/setter_params.hpp"
#include "modules/runtime/tests/py_param_server_test_helper.hpp"
#include "modules/commons/transformation/frenet.hpp"

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


    m.def("ParamsTest", &DoSomeParams);

    py::class_<CppParamServerTestObject, std::shared_ptr<CppParamServerTestObject>>(m, "CppParamServerTestObject")
      .def(py::init<std::shared_ptr<modules::commons::Params>>())
      .def("GetRealValue", &CppParamServerTestObject::GetRealValue)
      .def("GetBoolValueTrue", &CppParamServerTestObject::GetBoolValueTrue)
      .def("GetBoolValueFalse", &CppParamServerTestObject::GetBoolValueFalse)
      .def("GetIntValue", &CppParamServerTestObject::GetIntValue)
      .def("GetListListFloatValue", &CppParamServerTestObject::GetListListFloatValue)
      .def("GetListFloatValue", &CppParamServerTestObject::GetListFloatValue)
      .def("GetParams", &CppParamServerTestObject::GetParams)
      .def(py::pickle(
      [](const CppParamServerTestObject& p) -> py::tuple {
        return py::make_tuple(ParamsToPython(p.GetParams()));
      },
      [](py::tuple  &t)  {
          auto params_ptr = PythonToParams(t[0].cast<py::tuple>());
          return CppParamServerTestObject(params_ptr);
      }));


    py::class_<transformation::FrenetPosition, std::shared_ptr<transformation::FrenetPosition>>(m, "FrenetPosition")
      .def(py::init<>())
      .def_readwrite("lon", &transformation::FrenetPosition::lon)
      .def_readwrite("lat", &transformation::FrenetPosition::lat);

}

}  // namespace commons
}  // namespace modules
