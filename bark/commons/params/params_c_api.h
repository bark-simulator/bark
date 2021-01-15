// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// #ifdef __cplusplus
// #include "bark/commons/params/setter_params.hpp"
//#endif

typedef void* CSetterParams;

extern "C" CSetterParams newCSetterParams(bool log_if_default);
extern "C" void delCSetterParams(CSetterParams c_setter_params);
extern "C" double getRealCSetterParams(CSetterParams c_setter_params, const char* param_name,
                                       const char* description,
                                       double default_value);
extern "C" void setRealCSetterParams(CSetterParams c_setter_params, const char* param_name,
                                     double value);