// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_COMMONS_BASE_TYPE_HPP_
#define MODULES_COMMONS_BASE_TYPE_HPP_

#include "modules/commons/params/params.hpp"
#include "modules/commons/util/util.hpp"

namespace modules {
namespace commons {

class BaseType {
 public:
  explicit BaseType(ParamsPtr params) : params_(params) {}

  BaseType(const BaseType& base) : params_(base.params_) {}
  ~BaseType() {}

  ParamsPtr GetParams() const { return params_;}

 private:
  ParamsPtr params_;  // do not own
};

}  // namespace commons
}  // namespace modules
#endif  // MODULES_COMMONS_BASE_TYPE_HPP_
