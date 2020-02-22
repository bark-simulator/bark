// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_PARAMS_SETTER_PARAMS_HPP_
#define MODULES_COMMONS_PARAMS_SETTER_PARAMS_HPP_

#include <string>
#include <unordered_map>
#include "modules/commons/params/params.hpp"
#include "modules/commons/util/util.hpp"
namespace modules {
namespace commons {

// This class is mainly useful for test definitions in C++
class SetterParams : public Params {
 public:
  SetterParams(bool log_if_default = false)
      : params_bool_(),
        params_real_(),
        params_int_(),
        params_listlist_float_(),
        params_list_float_(),
        log_if_default_(log_if_default) {}
  SetterParams(bool log_if_default, const CondensedParamList &param_list);

  virtual ~SetterParams() {}

  // get and set parameters as in python
  virtual bool GetBool(const std::string &param_name,
                       const std::string &description,
                       const bool &default_value) {
    return get_parameter(params_bool_, param_name, default_value);
  }

  virtual float GetReal(const std::string &param_name,
                        const std::string &description,
                        const float &default_value) {
    return get_parameter(params_real_, param_name, default_value);
  }

  virtual int GetInt(const std::string &param_name,
                     const std::string &description, const int &default_value) {
    return get_parameter(params_int_, param_name, default_value);
  }

  virtual std::vector<std::vector<float>> GetListListFloat(
      const std::string &param_name, const std::string &description,
      const std::vector<std::vector<float>> &default_value) {
    return get_parameter(params_listlist_float_, param_name, default_value);
  }

  virtual std::vector<float> GetListFloat(
      const std::string &param_name, const std::string &description,
      const std::vector<float> &default_value) {
    return get_parameter(params_list_float_, param_name, default_value);
  }

  // not used atm
  virtual void SetBool(const std::string &param_name, const bool &value) {
    set_parameter(params_bool_, param_name, value);
  }
  virtual void SetReal(const std::string &param_name, const float &value) {
    set_parameter(params_real_, param_name, value);
  }
  virtual void SetInt(const std::string &param_name, const int &value) {
    set_parameter(params_int_, param_name, value);
  }
  virtual void SetListListFloat(const std::string &param_name,
                                const std::vector<std::vector<float>> &value) {
    set_parameter(params_listlist_float_, param_name, value);
  }
  virtual void SetListFloat(const std::string &param_name,
                            const std::vector<float> &value) {
    set_parameter(params_list_float_, param_name, value);
  }

  virtual CondensedParamList GetCondensedParamList() const;  // < not needed atm

  virtual int operator[](const std::string &param_name) {
    throw;
  }  //< not supported atm

  virtual ParamsPtr AddChild(const std::string &name) {
    const auto it = childs_.find(name);
    if (it != childs_.end()) {
      return it->second;
    }

    std::shared_ptr<SetterParams> child(new SetterParams(log_if_default_));
    childs_[name] = child;
    return child;
  }

 private:
  template <typename T>
  std::unordered_map<std::string, T> &get_param_map();

  template <typename M, typename T>
  void set_parameter(M &map, std::string param_name, T value) {
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
  T get_parameter(M map, std::string param_name, const T &default_value) {
    const auto it = map.find(param_name);
    if (it != map.end()) {
      return it->second;
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
        return child_param->get_parameter(child_param->get_param_map<T>(),
                                          child_param_name, default_value);
      }
      if (log_if_default_) {
        LOG(WARNING) << "Using default " << default_value << " for param \""
                     << param_name << "\"";
      }
      return default_value;
    }
  }

  std::unordered_map<std::string, std::shared_ptr<SetterParams>> childs_;
  std::unordered_map<std::string, bool> params_bool_;
  std::unordered_map<std::string, float> params_real_;
  std::unordered_map<std::string, int> params_int_;
  std::unordered_map<std::string, std::vector<std::vector<float>>>
      params_listlist_float_;
  std::unordered_map<std::string, std::vector<float>> params_list_float_;

  bool log_if_default_;
};

struct ParamVisitor : public boost::static_visitor<> {
  ParamVisitor(SetterParams *params, const std::string &param_name)
      : params_(params), param_name_(param_name) {}
  void operator()(bool b) const { params_->SetBool(param_name_, b); }
  void operator()(float r) const { params_->SetReal(param_name_, r); }
  void operator()(int i) const { params_->SetInt(param_name_, i); }
  void operator()(const ListListFloat &l) const {
    params_->SetListListFloat(param_name_, l);
  }
  void operator()(const ListFloat &l) const {
    params_->SetListFloat(param_name_, l);
  }

 private:
  SetterParams *params_;
  const std::string &param_name_;
};

template <>
inline std::unordered_map<std::string, bool> &SetterParams::get_param_map() {
  return params_bool_;
}

template <>
inline std::unordered_map<std::string, float> &SetterParams::get_param_map() {
  return params_real_;
}

template <>
inline std::unordered_map<std::string, int> &SetterParams::get_param_map() {
  return params_int_;
}

template <>
inline std::unordered_map<std::string, std::vector<std::vector<float>>>
    &SetterParams::get_param_map() {
  return params_listlist_float_;
}

template <>
inline std::unordered_map<std::string, std::vector<float>>
    &SetterParams::get_param_map() {
  return params_list_float_;
}

}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_PARAMS_SETTER_PARAMS_HPP_
