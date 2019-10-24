// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_
#define MODULES_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_

#include <Eigen/Core>
#include <boost/variant.hpp>
#include <queue>
#include <memory>
#include <utility>

#include "modules/commons/base_type.hpp"

namespace modules {
namespace models {
namespace dynamic {

typedef enum StateDefinition : int {
  TIME_POSITION = 0,  // unit is seconds
  X_POSITION = 1,  // unit is meter
  Y_POSITION = 2,  // unit is meter
  THETA_POSITION = 3,  // unit is rad
  VEL_POSITION = 4,  // unit is meter/second
  MIN_STATE_SIZE = 5,
  Z_POSITION = 6  // only placeholder, not used at the moment
} StateDefinition;
using State = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using Input = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Trajectory = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;


class DynamicModel : public commons::BaseType {
 public:
  explicit DynamicModel(modules::commons::Params *params) :
    BaseType(params), input_size_(0) {}

  virtual ~DynamicModel() {}

  virtual State StateSpaceModel(const State &x, const Input &u) const = 0;

  virtual std::shared_ptr<DynamicModel> Clone() const = 0;

  int input_size_;
};

typedef std::shared_ptr<DynamicModel> DynamicModelPtr;

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_DYNAMIC_MODEL_HPP_
