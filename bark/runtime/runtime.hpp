// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_RUNTIME_RUNTIME_HPP_
#define BARK_RUNTIME_RUNTIME_HPP_

#include <memory>

#include "bark/commons/commons.hpp"

namespace bark {
namespace runtime {

class Runtime : public bark::commons::BaseType {
 public:
  explicit Runtime(const commons::ParamsPtr& params)
      : commons::BaseType(params) {}

  Runtime(const Runtime& runtime) : commons::BaseType(runtime.GetParams()) {}

  virtual ~Runtime() {}

  virtual void Step() {}
  virtual void Step(int action) {}
  virtual void Step(float action) {}
  virtual void Step(double action) {}
  virtual void Step(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> m) {}
};

inline void EvalRuntime(
    Runtime r, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> action) {
  LOG(INFO) << "Received valid runtime." << std::endl;
  LOG(INFO) << "Stepping runtime..." << std::endl;
  r.Step(action);
  LOG(INFO) << "Runtime has been successfully stepped." << std::endl;
}

inline void EvalRuntime(Runtime r, int action) {
  LOG(INFO) << "Received valid runtime." << std::endl;
  LOG(INFO) << "Stepping runtime..." << std::endl;
  r.Step(action);
  LOG(INFO) << "Runtime has been successfully stepped." << std::endl;
}

typedef std::shared_ptr<Runtime> RuntimePtr;

}  // namespace runtime
}  // namespace bark

#endif  // BARK_RUNTIME_RUNTIME_HPP_
