// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
#define MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_

#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace dynamic {

class SingleTrackModel : public DynamicModel {
 public:
  explicit SingleTrackModel(modules::commons::Params *params) :
    DynamicModel(params),
    wheel_base_(2.7) {
      wheel_base_ = params->get_real("DynamicModel::wheel_base",
      "Wheel base of vehicle.",
      2.7);
    }
  virtual ~SingleTrackModel() {}

  State StateSpaceModel(const State &x, const Input &u) const {
    // TODO(@fortiss): get parameters from Params
    State tmp(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    tmp << 1,
    x(StateDefinition::VEL_POSITION) * cos(x(StateDefinition::THETA_POSITION)),
    x(StateDefinition::VEL_POSITION) * sin(x(StateDefinition::THETA_POSITION)),
    x(StateDefinition::VEL_POSITION) * tan(u(1)) / wheel_base_,
    u(0);
    return tmp;
  }

  std::shared_ptr<DynamicModel> Clone() const {
    std::shared_ptr<SingleTrackModel> model_ptr =
      std::make_shared<SingleTrackModel>(*this);
    return std::dynamic_pointer_cast<DynamicModel>(model_ptr);
  }

 private:
  float wheel_base_;
};

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
