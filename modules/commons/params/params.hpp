// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_PARAMS_PARAMS_HPP_
#define MODULES_COMMONS_PARAMS_PARAMS_HPP_

#include <string>
#include <vector>

namespace modules {
namespace commons {


class Params {
 public:
  Params() {}

  virtual ~Params() {}

  // get and set parameters as in python
  virtual bool get_bool(const std::string &param_name,
                        const std::string &description,
                        const bool &default_value) = 0;

  virtual float get_real(const std::string &param_name,
                         const std::string &description,
                         const float &default_value) = 0;

  virtual int get_int(const std::string &param_name,
                      const std::string &description,
                      const int &default_value) = 0;

  virtual std::vector<std::vector<float>> get_listlist_float(
                      const std::string &param_name,
                      const std::string &description,
                      const std::vector<std::vector<float>> &default_value) = 0;

  virtual void set_bool(const std::string &param_name, const bool &value) = 0;
  virtual void set_real(const std::string &param_name, const float &value) = 0;
  virtual void set_int(const std::string &param_name, const int &value) = 0;
  virtual void set_listlist_float(
    const std::string &param_name,
    const std::vector<std::vector<float>> &value) = 0;
  virtual int operator[](const std::string &param_name) = 0;
  virtual Params* AddChild(const std::string &name) = 0;
};


}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_PARAMS_PARAMS_HPP_
