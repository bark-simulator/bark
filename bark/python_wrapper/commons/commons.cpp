// Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "commons.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/runtime/tests/py_param_server_test_helper.hpp"
#include "bark/commons/transformation/frenet.hpp"

namespace py = pybind11;

namespace bark {
namespace commons {

void python_commons(py::module m) {
    py::class_<Params, PyParams, std::shared_ptr<Params>>(m, "Params")
      .def(py::init<>())
      .def("Access", &Params::operator[])
      .def("addChild", &Params::AddChild)
      .def("getBool", &Params::GetBool)
      .def("getReal", &Params::GetReal)
      .def("getInt", &Params::GetInt)
      .def("getString", &Params::GetString)
      .def("setBool", &Params::SetBool)
      .def("setReal", &Params::SetReal)
      .def("setString", &Params::SetString)
      .def("getCondensedParamList", &Params::GetCondensedParamList)
      .def("setInt", &Params::SetInt);


    m.def("ParamsTest", &DoSomeParams);

    py::class_<CppParamServerTestObject, std::shared_ptr<CppParamServerTestObject>>(m, "CppParamServerTestObject")
      .def(py::init<std::shared_ptr<bark::commons::Params>>())
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

    m.def("SetLogLevel", [](int level) { FLAGS_minloglevel = level; });
    m.def("SetVerboseLevel", [](int level) { FLAGS_v = level; });
}

}  // namespace commons
}  // namespace bark
