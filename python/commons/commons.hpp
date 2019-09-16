// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef PYTHON_PYTHON_BINDINGS_COMMONS_COMMONS_HPP_
#define PYTHON_PYTHON_BINDINGS_COMMONS_COMMONS_HPP_

#include <string>

#include "python/common.hpp"
#include "modules/commons/params/params.hpp"
#include "modules/commons/params/params_test.h"

namespace py = pybind11;

namespace modules {
namespace commons {

class PyParams : public Params {
 public:
  using Params::Params;

  int operator[](const std::string &param_name) override {
    PYBIND11_OVERLOAD_PURE(
        int,
        Params,
        access,
        param_name);
  }

  bool get_bool(const std::string &param_name, const std::string &description, const bool &default_value) override {
    PYBIND11_OVERLOAD_PURE(
        bool,
        Params,
        get_bool,
        param_name, description, default_value);
  }

  float get_real(const std::string &param_name, const std::string &description, const float &default_value) override {
    PYBIND11_OVERLOAD_PURE(
        float,
        Params,
        get_real,
        param_name, description, default_value);
  }

  int get_int(const std::string &param_name, const std::string &description, const int &default_value) override {
    PYBIND11_OVERLOAD_PURE(
        int,
        Params,
        get_int,
        param_name, description, default_value);
  }

  std::vector<std::vector<float>> get_listlist_float(const std::string &param_name, const std::string &description, const std::vector<std::vector<float>> &default_value) override {
    PYBIND11_OVERLOAD_PURE(
        std::vector<std::vector<float>>,
        Params,
        get_listlist_float,
        param_name, description, default_value);
  }

  void set_bool(const std::string &param_name, const bool &value) override {
    PYBIND11_OVERLOAD_PURE(
        void,
        Params,
        set_bool,
        param_name, value);
  }

  void set_real(const std::string &param_name, const float &value) override {
    PYBIND11_OVERLOAD_PURE(
        void,
        Params,
        set_real,
        param_name, value);
  }

  void set_int(const std::string &param_name, const int &value) override {
    PYBIND11_OVERLOAD_PURE(
        void,
        Params,
        set_int,
        param_name, value);
  }

  void set_listlist_float(const std::string &param_name, const std::vector<std::vector<float>> &value) override {
    PYBIND11_OVERLOAD_PURE(
        void,
        Params,
        set_listlist_float,
        param_name, value);
  }

  Params *AddChild(const std::string &name) override {
    PYBIND11_OVERLOAD_PURE(
        Params *,
        Params,
        AddChild,
        name);
  }
};

void python_commons(py::module m);

}  // namespace commons
}  // namespace modules

#endif  // PYTHON_PYTHON_BINDINGS_COMMONS_COMMONS_HPP_
