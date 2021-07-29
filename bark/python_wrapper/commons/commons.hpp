// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef PYTHON_PYTHON_BINDINGS_COMMONS_COMMONS_HPP_
#define PYTHON_PYTHON_BINDINGS_COMMONS_COMMONS_HPP_

#include <string>
#include <unordered_map>

#include "bark/commons/params/params.hpp"
#include "bark/commons/params/params_test.h"
#include "bark/python_wrapper/common.hpp"

namespace py = pybind11;

namespace bark {
namespace commons {

class PyParams : public Params {
 public:
  using Params::Params;

  int operator[](const std::string& param_name) override {
    PYBIND11_OVERLOAD_PURE(int, Params, access, param_name);
  }

  bool GetBool(const std::string& param_name, const std::string& description,
               const bool& default_value) override {
    PYBIND11_OVERLOAD_PURE(bool, Params, GetBool, param_name, description,
                           default_value);
  }

  double GetReal(const std::string& param_name, const std::string& description,
                const double& default_value) override {
    PYBIND11_OVERLOAD_PURE(double, Params, GetReal, param_name, description,
                           default_value);
  }

  int GetInt(const std::string& param_name, const std::string& description,
             const int& default_value) override {
    PYBIND11_OVERLOAD_PURE(int, Params, GetInt, param_name, description,
                           default_value);
  }

  std::string GetString(const std::string& param_name,
                        const std::string& description,
                        const std::string& default_value) override {
    PYBIND11_OVERLOAD_PURE(std::string, Params, GetString, param_name,
                           description, default_value);
  }

  std::vector<std::vector<double>> GetListListFloat(
      const std::string& param_name, const std::string& description,
      const std::vector<std::vector<double>>& default_value) override {
    PYBIND11_OVERLOAD_PURE(std::vector<std::vector<double>>, Params,
                           GetListListFloat, param_name, description,
                           default_value);
  }

  std::vector<double> GetListFloat(
      const std::string& param_name, const std::string& description,
      const std::vector<double>& default_value) override {
    PYBIND11_OVERLOAD_PURE(std::vector<double>, Params, GetListFloat, param_name,
                           description, default_value);
  }

  std::vector<int> GetListInt(
      const std::string& param_name, const std::string& description,
      const std::vector<int>& default_value) override {
    PYBIND11_OVERLOAD_PURE(std::vector<int>, Params, GetListInt, param_name,
                           description, default_value);
  }

  void SetBool(const std::string& param_name, const bool& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetBool, param_name, value);
  }

  void SetReal(const std::string& param_name, const double& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetReal, param_name, value);
  }

  void SetInt(const std::string& param_name, const int& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetInt, param_name, value);
  }

  void SetString(const std::string& param_name,
                 const std::string& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetString, param_name, value);
  }

  void SetListListFloat(const std::string& param_name,
                        const std::vector<std::vector<double>>& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetListListFloat, param_name, value);
  }

  void SetListFloat(const std::string& param_name,
                    const std::vector<double>& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetListFloat, param_name, value);
  }

  void SetListInt(const std::string& param_name,
                    const std::vector<int>& value) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetListInt, param_name, value);
  }

  void SetDistribution(const std::string& param_name,
                       const std::string& distribution_type) override {
    PYBIND11_OVERLOAD_PURE(void, Params, SetDistribution, param_name,
                           distribution_type);
  }

  ParamsPtr AddChild(const std::string& name) override {
    PYBIND11_OVERLOAD_PURE(ParamsPtr, Params, AddChild, name);
  }

  CondensedParamList GetCondensedParamList() const override {
    PYBIND11_OVERLOAD_PURE(CondensedParamList, Params, GetCondensedParamList, );
  }
};

void python_commons(py::module m);

}  // namespace commons
}  // namespace bark

#endif  // PYTHON_PYTHON_BINDINGS_COMMONS_COMMONS_HPP_
