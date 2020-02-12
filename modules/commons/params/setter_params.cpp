// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/commons/params/setter_params.hpp"

namespace modules {
namespace commons {

bool SetterParams::GetBool(const std::string& param_name,
                           const std::string& description,
                           const bool& default_value) {
  return get_parameter(params_bool_, param_name, default_value);
}

template <>
std::unordered_map<std::string, bool>& SetterParams::get_param_map() {
  return params_bool_;
}
float SetterParams::GetReal(const std::string& param_name,
                            const std::string& description,
                            const float& default_value) {
  return get_parameter(params_real_, param_name, default_value);
}
int SetterParams::GetInt(const std::string& param_name,
                         const std::string& description,
                         const int& default_value) {
  return get_parameter(params_int_, param_name, default_value);
}
std::vector<std::vector<float>> SetterParams::GetListListFloat(
    const std::string& param_name, const std::string& description,
    const std::vector<std::vector<float>>& default_value) {
  return get_parameter(params_listlist_float_, param_name, default_value);
}
void SetterParams::SetBool(const std::string& param_name, const bool& value) {
  set_parameter(params_bool_, param_name, value);
}
void SetterParams::SetReal(const std::string& param_name, const float& value) {
  set_parameter(params_real_, param_name, value);
}
void SetterParams::SetInt(const std::string& param_name, const int& value) {
  set_parameter(params_int_, param_name, value);
}
void SetterParams::SetListListFloat(
    const std::string& param_name,
    const std::vector<std::vector<float>>& value) {
  set_parameter(params_listlist_float_, param_name, value);
}
Params* SetterParams::AddChild(const std::string& name) {
  const auto it = childs_.find(name);
  if (it != childs_.end()) {
    return it->second.get();
  }

  std::shared_ptr<SetterParams> child(new SetterParams(log_if_default_));
  childs_[name] = child;
  return child.get();
}

template <>
std::unordered_map<std::string, float>& SetterParams::get_param_map() {
  return params_real_;
}
template <>
std::unordered_map<std::string, int>& SetterParams::get_param_map() {
  return params_int_;
}

template <>
std::unordered_map<std::string, std::vector<std::vector<float>>>&
SetterParams::get_param_map() {
  return params_listlist_float_;
}

}  // namespace commons
}  // namespace modules
