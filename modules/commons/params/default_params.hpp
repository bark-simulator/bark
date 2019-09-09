// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_PARAMS_DEFAULT_PARAMS_HPP_
#define MODULES_COMMONS_PARAMS_DEFAULT_PARAMS_HPP_

#include <string>
#include "modules/commons/params/params.hpp"

namespace modules {
namespace commons {

// This class is mainly useful for test definitions in C++
class DefaultParams : public Params {
 public:
  DefaultParams() {}

  virtual ~DefaultParams() {}

  // get and set parameters as in python
  virtual bool get_bool(const std::string &param_name,
                        const std::string &description,
                        const bool &default_value) { return default_value; }
                        
  virtual float get_real(const std::string &param_name,
                         const std::string &description,
                         const float &default_value) { return default_value; }

  virtual int get_int(const std::string &param_name,
                      const std::string &description,
                      const int &default_value) { return default_value; }

  virtual std::vector<std::vector<float>> get_listlist_float(const std::string &param_name,
                      const std::string &description,
                      const std::vector<std::vector<float>> &default_value)  { return default_value; }

  // not used atm
  virtual void set_bool(const std::string &param_name, const bool &value) {}
  virtual void set_real(const std::string &param_name, const float &value) {}
  virtual void set_int(const std::string &param_name, const int &value) {}
  virtual void set_listlist_float(const std::string &param_name,
                      const std::vector<std::vector<float>> &value) {}

  virtual int operator[](const std::string &param_name) { return 0; }

  virtual Params *AddChild(const std::string &name) { return this; }
};

}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_PARAMS_DEFAULT_PARAMS_HPP_
