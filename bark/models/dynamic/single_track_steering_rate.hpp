// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_DYNAMIC_SINGLE_TRACK_STEERING_RATE_HPP_
#define BARK_MODELS_DYNAMIC_SINGLE_TRACK_STEERING_RATE_HPP_

#include <algorithm>
#include <memory>
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace dynamic {


class SingleTrackSteeringRateModel : public DynamicModel {
 public:
  explicit SingleTrackSteeringRateModel(const bark::commons::ParamsPtr& params)
      : DynamicModel(params),
        wheel_base_(params->GetReal("DynamicModel::WheelBase",
                                    "Wheel base of vehicle [m]", 2.7)){
  }

  virtual ~SingleTrackSteeringRateModel() {}

  State StateSpaceModel(const State& x, const Input& u) const {
    State tmp(static_cast<int>(StateDefinition::MIN_STATE_SIZE ) + 1);
    tmp << 1,
        x(StateDefinition::VEL_POSITION) *
            cos(x(StateDefinition::THETA_POSITION)),
        x(StateDefinition::VEL_POSITION) *
            sin(x(StateDefinition::THETA_POSITION)),
        x(StateDefinition::VEL_POSITION) * tan(
          x[5]) / wheel_base_,
        u(0),
        u(1);
    return tmp;
  }

  std::shared_ptr<DynamicModel> Clone() const {
    std::shared_ptr<SingleTrackSteeringRateModel> model_ptr =
        std::make_shared<SingleTrackSteeringRateModel>(*this);
    return std::dynamic_pointer_cast<DynamicModel>(model_ptr);
  }

  virtual int GetStateSize() const {
    return static_cast<int>(StateDefinition::MIN_STATE_SIZE) + 1;
  }

  double GetWheelBase() const { return wheel_base_; }

 private:
  double wheel_base_;
};

}  // namespace dynamic
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_DYNAMIC_SINGLE_TRACK_STEERING_RATE_HPP_
