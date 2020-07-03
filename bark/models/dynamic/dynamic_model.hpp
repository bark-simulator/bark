// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_
#define BARK_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_

#include <Eigen/Core>
#include <boost/variant.hpp>
#include <memory>
#include <queue>
#include <utility>

#include "bark/commons/commons.hpp"

namespace bark {
namespace models {
namespace dynamic {

typedef enum StateDefinition : int {
  TIME_POSITION = 0,   // unit is seconds
  X_POSITION = 1,      // unit is meter
  Y_POSITION = 2,      // unit is meter
  THETA_POSITION = 3,  // unit is rad
  VEL_POSITION = 4,    // unit is meter/second
  MIN_STATE_SIZE = 5,
  Z_POSITION = 6,
  INP_ACC = 7,   // acceleration input [m/s^2]
  INP_DELTA = 8  // steerint-rate input [m/s]
} StateDefinition;

using State = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using Input = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Trajectory = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

inline bool IsValid(const State& state) {
  return ((state - state).array() == (state - state).array()).all() &&
         ((state.array() == state.array())).all();
}

inline bool IsValid(const Trajectory& state) {
  return ((state - state).array() == (state - state).array()).all() &&
         ((state.array() == state.array())).all();
}

class DynamicModel : public commons::BaseType {
 public:
  explicit DynamicModel(bark::commons::ParamsPtr params)
      : BaseType(params), input_size_(0) {}

  virtual ~DynamicModel() {}

  virtual State StateSpaceModel(const State& x, const Input& u) const = 0;

  virtual std::shared_ptr<DynamicModel> Clone() const = 0;

  int input_size_;
};

typedef std::shared_ptr<DynamicModel> DynamicModelPtr;

}  // namespace dynamic
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_
