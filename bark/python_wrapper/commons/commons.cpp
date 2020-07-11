// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "commons.hpp"
#include "bark/commons/base_type.hpp"
#include "bark/commons/params/setter_params.hpp"
#include "bark/commons/transformation/frenet.hpp"
#include "bark/python_wrapper/polymorphic_conversion.hpp"
#include "bark/python_wrapper/tests/logging_tests.hpp"
#include "bark/runtime/tests/py_param_server_test_helper.hpp"
#include "glog/logging.h"

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
      .def("getListFloat", &Params::GetListFloat)
      .def("getListListFloat", &Params::GetListListFloat)
      .def("getString", &Params::GetString)
      .def("setBool", &Params::SetBool)
      .def("setReal", &Params::SetReal)
      .def("setString", &Params::SetString)
      .def("getCondensedParamList", &Params::GetCondensedParamList)
      .def("setInt", &Params::SetInt);

  m.def("ParamsTest", &DoSomeParams);

  py::class_<CppParamServerTestObject,
             std::shared_ptr<CppParamServerTestObject>>(
      m, "CppParamServerTestObject")
      .def(py::init<std::shared_ptr<bark::commons::Params>>())
      .def("GetRealValue", &CppParamServerTestObject::GetRealValue)
      .def("GetBoolValueTrue", &CppParamServerTestObject::GetBoolValueTrue)
      .def("GetBoolValueFalse", &CppParamServerTestObject::GetBoolValueFalse)
      .def("GetIntValue", &CppParamServerTestObject::GetIntValue)
      .def("GetListListFloatValue",
           &CppParamServerTestObject::GetListListFloatValue)
      .def("GetListFloatValue", &CppParamServerTestObject::GetListFloatValue)
      .def("GetParams", &CppParamServerTestObject::GetParams)
      .def(py::pickle(
          [](const CppParamServerTestObject& p) -> py::tuple {
            return py::make_tuple(ParamsToPython(p.GetParams()));
          },
          [](py::tuple& t) {
            auto params_ptr = PythonToParams(t[0].cast<py::tuple>());
            return CppParamServerTestObject(params_ptr);
          }));

  py::class_<transformation::FrenetPosition,
             std::shared_ptr<transformation::FrenetPosition>>(m,
                                                              "FrenetPosition")
      .def(py::init<>())
      .def_readwrite("lon", &transformation::FrenetPosition::lon)
      .def_readwrite("lat", &transformation::FrenetPosition::lat);

  m.def("SetLogLevel", [](int level) { FLAGS_minloglevel = level; });
  m.def("SetVerboseLevel", [](int level) { FLAGS_v = level; });

  py::class_<BaseType, std::shared_ptr<BaseType>>(m, "BaseType")
      .def(py::init<const ParamsPtr&>())
      .def_property_readonly("params", &BaseType::GetParams);

  m.def(
      "GLogInit",
      [](char* program_path, char* log_path, int v_level, bool log_to_std_err) {
        FLAGS_v = v_level;
        FLAGS_alsologtostderr = log_to_std_err;
        FLAGS_log_dir = log_path;
        FLAGS_minloglevel = 0;
        google::InitGoogleLogging(program_path);
        LOG(INFO) << "GLog init";
      },
      py::arg("program_path") = "", py::arg("log_path") = "/tmp",
      py::arg("v_level") = 0, py::arg("log_to_std_err") = true);

  m.def("do_logging", &do_logging);
}

}  // namespace commons
}  // namespace bark
