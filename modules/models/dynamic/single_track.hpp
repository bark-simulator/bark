// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
#define MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_

#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/models/dynamic/integration.hpp"

namespace modules {
namespace models {
namespace dynamic {

class SingleTrackModel : public DynamicModel {
 public:
  SingleTrackModel() {}
  virtual ~SingleTrackModel() {}

  State StateSpaceModel(const State &x, const Input &u) const {
    // TODO(@fortiss): get parameters from Params
    float Lf = 2.6;
    State tmp(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    tmp << 1,
    x(StateDefinition::VEL_POSITION) * cos(x(StateDefinition::THETA_POSITION)),
    x(StateDefinition::VEL_POSITION) * sin(x(StateDefinition::THETA_POSITION)),
    tan(u(1)) / Lf,
    u(0);
    return tmp;
  }

  DynamicModel *Clone() const {
    return new SingleTrackModel(*this);
  }
};

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
