// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_RUNTIME_RUNTIME_HPP_
#define MODULES_RUNTIME_RUNTIME_HPP_

#include <memory>

#include "modules/commons/base_type.hpp"

namespace modules {
namespace runtime {


class Runtime : public modules::commons::BaseType {
 public:
  explicit Runtime(commons::Params *params) : commons::BaseType(params) {}

  Runtime(const Runtime &runtime) : 
              commons::BaseType(runtime.get_params()) {}

  virtual ~Runtime() {}

  virtual void Step() {}

};

typedef std::shared_ptr<Runtime> RuntimePtr;

}  // namespace runtime
}  // namespace modules

#endif  // MODULES_RUNTIME_RUNTIME_HPP_
