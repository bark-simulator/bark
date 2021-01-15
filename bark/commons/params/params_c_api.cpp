// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/params/params_c_api.h"
#include "bark/commons/params/setter_params.hpp"

using bark::commons::SetterParams;

CSetterParams newCSetterParams(bool log_if_default) {
  return reinterpret_cast<void*>(new SetterParams(log_if_default));
}

void delCSetterParams(CSetterParams c_setter_params) {
  delete reinterpret_cast<SetterParams*>(c_setter_params);
}

double getRealCSetterParams(CSetterParams c_setter_params,
                            const char* param_name, const char* description,
                            double default_value) {
  return reinterpret_cast<SetterParams*>(c_setter_params)
      ->GetReal(param_name, description, default_value);
}

void setRealCSetterParams(CSetterParams c_setter_params, const char* param_name,
                          double value) {
  return reinterpret_cast<SetterParams*>(c_setter_params)
      ->SetReal(param_name, value);
}