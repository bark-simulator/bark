// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_PARAMS_PARAMS_HPP_
#define MODULES_COMMONS_PARAMS_PARAMS_HPP_

#include <string>
#include <vector>
#include "boost/variant.hpp"

namespace modules {
namespace commons {

typedef std::vector<std::vector<float>> ListListFloat;
typedef std::pair<std::string, boost::variant<bool, float, int, ListListFloat>> ParamPair;
typedef std::vector<ParamPair> CondensedParamList;

class Params {
 public:
  Params() {}

  virtual ~Params() {}

  // get and set parameters as in python
  virtual bool GetBool(const std::string &param_name,
                        const std::string &description,
                        const bool &default_value) = 0;

  virtual float GetReal(const std::string &param_name,
                         const std::string &description,
                         const float &default_value) = 0;

  virtual int GetInt(const std::string &param_name,
                      const std::string &description,
                      const int &default_value) = 0;

  virtual std::vector<std::vector<float>> GetListListFloat(
                      const std::string &param_name,
                      const std::string &description,
                      const ListListFloat &default_value) = 0;

  virtual CondensedParamList GetCondensedParamList() = 0;


  virtual void SetBool(const std::string &param_name, const bool &value) = 0;
  virtual void SetReal(const std::string &param_name, const float &value) = 0;
  virtual void SetInt(const std::string &param_name, const int &value) = 0;
  virtual void SetListListFloat(
    const std::string &param_name,
    const ListListFloat &value) = 0;
  virtual int operator[](const std::string &param_name) = 0;
  virtual Params* AddChild(const std::string &name) = 0;
};


}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_PARAMS_PARAMS_HPP_
