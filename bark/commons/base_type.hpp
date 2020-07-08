// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_BASE_TYPE_HPP_
#define BARK_COMMONS_BASE_TYPE_HPP_

#include "bark/commons/util/util.hpp"

namespace bark {
namespace commons {

class Params;
typedef std::shared_ptr<Params> ParamsPtr;

class BaseType {
 public:
  explicit BaseType(ParamsPtr params) : params_(params) {}

  BaseType(const BaseType& base) : params_(base.params_) {}
  ~BaseType() {}

  ParamsPtr GetParams() const { return params_; }

 private:
  ParamsPtr params_;  // do not own
};

}  // namespace commons
}  // namespace bark
#endif  // BARK_COMMONS_BASE_TYPE_HPP_
