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

  Runtime(const Runtime& runtime) :
              commons::BaseType(runtime.get_params()) {}

  virtual ~Runtime() {}

  virtual void Step() {}
  virtual void Step(int action) {}
  virtual void Step(float action) {}
  virtual void Step(double action) {}
  virtual void Step(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> m) {}

};

inline void EvalRuntime(Runtime r,
                        Eigen::Matrix<float,
                                      Eigen::Dynamic,
                                      Eigen::Dynamic> action) {
  LOG(INFO)<< "Received valid runtime." << std::endl;
  LOG(INFO) << "Stepping runtime..." << std::endl;
  r.Step(action);
  LOG(INFO) << "Runtime has been successfully stepped." << std::endl;
}

inline void EvalRuntime(Runtime r,
                        int action) {
  LOG(INFO) << "Received valid runtime." << std::endl;
  LOG(INFO) << "Stepping runtime..." << std::endl;
  r.Step(action);
  LOG(INFO) << "Runtime has been successfully stepped." << std::endl;
}

typedef std::shared_ptr<Runtime> RuntimePtr;

}  // namespace runtime
}  // namespace modules

#endif  // MODULES_RUNTIME_RUNTIME_HPP_
