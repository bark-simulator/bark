// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_PARAMS_SETTER_PARAMS_HPP_
#define BARK_COMMONS_PARAMS_SETTER_PARAMS_HPP_

#include <string>
#include <unordered_map>
#include "bark/commons/params/params.hpp"
#include "bark/commons/util/util.hpp"
namespace bark {
namespace commons {

class SetterParams : public Params {
 public:
  SetterParams(bool log_if_default = false)
      : params_bool_(),
        params_real_(),
        params_int_(),
        params_listlist_float_(),
        params_list_float_(),
        log_if_default_(log_if_default) {}
  SetterParams(bool log_if_default, const CondensedParamList& param_list);

  virtual ~SetterParams() {}

  // get and set parameters as in python
  virtual bool GetBool(const std::string& param_name,
                       const std::string& description,
                       const bool& default_value) {
    return get_parameter(params_bool_, param_name, default_value);
  }

  virtual float GetReal(const std::string& param_name,
                        const std::string& description,
                        const float& default_value) {
    return get_parameter(params_real_, param_name, default_value);
  }

  virtual int GetInt(const std::string& param_name,
                     const std::string& description, const int& default_value) {
    return get_parameter(params_int_, param_name, default_value);
  }

  virtual std::string GetString(const std::string& param_name,
                                const std::string& description,
                                const std::string& default_value) {
    return get_parameter(params_string_, param_name, default_value);
  }
  virtual std::vector<std::vector<float>> GetListListFloat(
      const std::string& param_name, const std::string& description,
      const std::vector<std::vector<float>>& default_value) {
    return get_parameter(params_listlist_float_, param_name, default_value);
  }

  virtual std::vector<float> GetListFloat(
      const std::string& param_name, const std::string& description,
      const std::vector<float>& default_value) {
    return get_parameter(params_list_float_, param_name, default_value);
  }

  virtual void SetBool(const std::string& param_name, const bool& value) {
    set_parameter(params_bool_, param_name, value);
  }
  virtual void SetReal(const std::string& param_name, const float& value) {
    set_parameter(params_real_, param_name, value);
  }
  virtual void SetInt(const std::string& param_name, const int& value) {
    set_parameter(params_int_, param_name, value);
  }
  virtual void SetString(const std::string& param_name,
                         const std::string& default_value) {
    set_parameter(params_string_, param_name, default_value);
  }
  virtual void SetListListFloat(const std::string& param_name,
                                const std::vector<std::vector<float>>& value) {
    set_parameter(params_listlist_float_, param_name, value);
  }
  virtual void SetListFloat(const std::string& param_name,
                            const std::vector<float>& value) {
    set_parameter(params_list_float_, param_name, value);
  }

  virtual void SetDistribution(const std::string& param_name,
                               const std::string& distribution_type) {
    auto param_name_distribution_type = param_name + "::" + "DistributionType";
    set_parameter(params_string_, param_name_distribution_type,
                  distribution_type);
  }

  virtual CondensedParamList GetCondensedParamList() const;

  virtual int operator[](const std::string& param_name) {
    throw;
  }  //< not supported atm

  virtual ParamsPtr AddChild(const std::string& name) {
    std::string delimiter = "::";
    // Try to resolve hierarchy
    auto child_name = name;
    std::string rest_name = "";

    auto pos = child_name.find(delimiter);
    if (pos != std::string::npos) {
      child_name = name.substr(0, pos);
      rest_name = name;
      rest_name.erase(0, pos + delimiter.length());
    }

    const auto it = childs_.find(child_name);
    std::shared_ptr<SetterParams> child;
    if (it != childs_.end()) {
      child = it->second;
    } else {
      child = std::make_shared<SetterParams>(log_if_default_);
      childs_[child_name] = child;
    }
    if (rest_name.empty()) {
      return child;
    } else {
      return child->AddChild(rest_name);
    }
  }

 private:
  template <typename T>
  std::unordered_map<std::string, T>& get_param_map();

  template <typename M, typename T>
  void set_parameter(M& map, std::string param_name, T value) {
    // find first child search there
    std::string delimiter = "::";
    auto pos = param_name.find(delimiter);
    if (pos != std::string::npos) {
      std::string child_name = param_name.substr(0, pos);
      auto child_param =
          std::dynamic_pointer_cast<SetterParams>(this->AddChild(child_name));
      std::string child_param_name =
          param_name.erase(0, pos + delimiter.length());
      child_param->set_parameter(child_param->get_param_map<T>(),
                                 child_param_name, value);
      return;
    }
    map[param_name] = value;  // no child specification found, simply set value
  }

  template <typename M, typename T>
  T get_parameter(M map, std::string param_name, const T& default_value) {
    auto search_result =
        get_parameter_recursive(map, param_name, default_value);
    if (!search_result.second && log_if_default_) {
      LOG(FATAL) << "Using default " << default_value << " for param \""
                 << param_name << "\"";
    }
    return search_result.first;
  }

  template <typename M, typename T>
  std::pair<T, bool> get_parameter_recursive(M map, std::string param_name,
                                             const T& default_value) {
    const auto it = map.find(param_name);
    if (it != map.end()) {
      return std::make_pair(it->second, true);
    } else {
      // find first child search there
      std::string delimiter = "::";
      auto pos = param_name.find(delimiter);
      if (pos != std::string::npos) {
        std::string child_name = param_name.substr(0, pos);
        auto child_param =
            std::dynamic_pointer_cast<SetterParams>(this->AddChild(child_name));
        std::string child_param_name =
            param_name.erase(0, pos + delimiter.length());
        return child_param->get_parameter_recursive(
            child_param->get_param_map<T>(), child_param_name, default_value);
      }
      return std::make_pair(default_value, false);
    }
  }

  std::unordered_map<std::string, std::shared_ptr<SetterParams>> childs_;
  std::unordered_map<std::string, bool> params_bool_;
  std::unordered_map<std::string, float> params_real_;
  std::unordered_map<std::string, int> params_int_;
  std::unordered_map<std::string, std::vector<std::vector<float>>>
      params_listlist_float_;
  std::unordered_map<std::string, std::vector<float>> params_list_float_;
  std::unordered_map<std::string, std::string> params_string_;

  bool log_if_default_;
};

struct ParamVisitor : public boost::static_visitor<> {
  ParamVisitor(SetterParams* params, const std::string& param_name)
      : params_(params), param_name_(param_name) {}
  void operator()(bool b) const { params_->SetBool(param_name_, b); }
  void operator()(float r) const { params_->SetReal(param_name_, r); }
  void operator()(int i) const { params_->SetInt(param_name_, i); }
  void operator()(const ListListFloat& l) const {
    params_->SetListListFloat(param_name_, l);
  }
  void operator()(const ListFloat& l) const {
    params_->SetListFloat(param_name_, l);
  }
  void operator()(const std::string& s) const {
    params_->SetString(param_name_, s);
  }

 private:
  SetterParams* params_;
  const std::string& param_name_;
};

template <>
inline std::unordered_map<std::string, bool>& SetterParams::get_param_map() {
  return params_bool_;
}

template <>
inline std::unordered_map<std::string, float>& SetterParams::get_param_map() {
  return params_real_;
}

template <>
inline std::unordered_map<std::string, int>& SetterParams::get_param_map() {
  return params_int_;
}

template <>
inline std::unordered_map<std::string, std::vector<std::vector<float>>>&
SetterParams::get_param_map() {
  return params_listlist_float_;
}

template <>
inline std::unordered_map<std::string, std::vector<float>>&
SetterParams::get_param_map() {
  return params_list_float_;
}

template <>
inline std::unordered_map<std::string, std::string>&
SetterParams::get_param_map() {
  return params_string_;
}

}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_PARAMS_SETTER_PARAMS_HPP_
