// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_
#define MODULES_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_

#include <Eigen/Core>
#include <queue>
#include <memory>
#include <utility>

namespace modules {
namespace models {
namespace dynamic {

typedef enum StateDefinition : int {
  TIME_POSITION = 0,
  X_POSITION = 1,
  Y_POSITION = 2,
  THETA_POSITION = 3,
  VEL_POSITION = 4,
  MIN_STATE_SIZE = 5,
  Z_POSITION = 6  // only placeholder, not used at the moment
} StateDefinition;
using State = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using Input = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Trajectory = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
typedef std::pair<State, Input> StateInputPair;
typedef std::queue<StateInputPair> StateInputHistory;

class DynamicModel {
 public:
  DynamicModel() {}

  virtual ~DynamicModel() {}

  virtual State StateSpaceModel(const State &x, const Input &u) const = 0;

  virtual DynamicModel *Clone() const = 0;
};

typedef std::shared_ptr<DynamicModel> DynamicModelPtr;

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_
