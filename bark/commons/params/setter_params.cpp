// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/commons/params/setter_params.hpp"

namespace bark {
namespace commons {

SetterParams::SetterParams(bool log_if_default,
                           const CondensedParamList& param_list)
    : log_if_default_(log_if_default) {
  for (const auto& param_pair : param_list) {
    const auto& param_name = param_pair.first;
    const auto& param_variant = param_pair.second;
    boost::apply_visitor(ParamVisitor(this, param_name), param_variant);
  }
}

CondensedParamList SetterParams::GetCondensedParamList() const {
  std::string hierarchy_delimiter = "::";
  CondensedParamList param_list;
  // Add Booleans
  for (const auto param : params_bool_) {
    param_list.push_back(std::make_pair(param.first, param.second));
  }

  // Add Ints
  for (const auto param : params_int_) {
    param_list.push_back(std::make_pair(param.first, param.second));
  }

  // Add Floats
  for (const auto param : params_real_) {
    param_list.push_back(std::make_pair(param.first, param.second));
  }

  // Add Strings
  for (const auto param : params_string_) {
    param_list.push_back(std::make_pair(param.first, param.second));
  }

  // Add List List Floats
  for (const auto param : params_listlist_float_) {
    param_list.push_back(std::make_pair(param.first, param.second));
  }

  // Add List Floats
  for (const auto param : params_list_float_) {
    param_list.push_back(std::make_pair(param.first, param.second));
  }

  // Add Childs recursively
  for (const auto& child : childs_) {
    CondensedParamList param_list_child = child.second->GetCondensedParamList();
    for (const auto& param_pair : param_list_child) {
      const std::string param_name =
          child.first + hierarchy_delimiter + param_pair.first;
      param_list.push_back(std::make_pair(param_name, param_pair.second));
    }
  }

  return param_list;
}

}  // namespace commons
}  // namespace bark