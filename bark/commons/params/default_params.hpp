// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_PARAMS_DEFAULT_PARAMS_HPP_
#define MODULES_COMMONS_PARAMS_DEFAULT_PARAMS_HPP_

#include <string>
#include "bark/commons/params/params.hpp"

namespace modules {
namespace commons {

// This class is mainly useful for test definitions in C++
class DefaultParams : public Params,
                      std::enable_shared_from_this<DefaultParams> {
 public:
  DefaultParams() {}

  virtual ~DefaultParams() {}

  // get and set parameters as in python
  virtual bool GetBool(const std::string &param_name,
                       const std::string &description,
                       const bool &default_value) {
    return default_value;
  }

  virtual float GetReal(const std::string &param_name,
                        const std::string &description,
                        const float &default_value) {
    return default_value;
  }

  virtual int GetInt(const std::string &param_name,
                     const std::string &description, const int &default_value) {
    return default_value;
  }

  virtual std::vector<std::vector<float>> GetListListFloat(
      const std::string &param_name, const std::string &description,
      const std::vector<std::vector<float>> &default_value) {
    return default_value;
  }

  virtual std::vector<float> GetListFloat(
      const std::string &param_name, const std::string &description,
      const std::vector<float> &default_value) {
    return default_value;
  }

  // not used atm
  virtual void SetBool(const std::string &param_name, const bool &value) {}
  virtual void SetReal(const std::string &param_name, const float &value) {}
  virtual void SetInt(const std::string &param_name, const int &value) {}
  virtual void SetListListFloat(const std::string &param_name,
                                const std::vector<std::vector<float>> &value) {}
  virtual void SetListFloat(const std::string &param_name,
                            const std::vector<float> &value) {}

  virtual int operator[](const std::string &param_name) { return 0; }

  virtual CondensedParamList GetCondensedParamList() const {
    throw;
  }  // < not needed atm

  virtual ParamsPtr AddChild(const std::string &name) {
    return shared_from_this();
  }  // < not needed atm
};

}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_PARAMS_DEFAULT_PARAMS_HPP_
