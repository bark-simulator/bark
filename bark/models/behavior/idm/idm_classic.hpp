// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
#define BARK_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_

#include <memory>
#include <tuple>

#include "bark/commons/transformation/frenet.hpp"
#include "bark/models/behavior/idm/base_idm.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;

// IDM that interpolates the vehicles using the center-line
// of a LaneCorridor
class BehaviorIDMClassic : public BaseIDM {
 public:
  explicit BehaviorIDMClassic(const commons::ParamsPtr& params)
      : BehaviorModel(params), BaseIDM(params) {}

  virtual ~BehaviorIDMClassic() {}

  std::tuple<Trajectory, Action> GenerateTrajectory(
      const world::ObservedWorld& observed_world,
      const LaneCorridorPtr& lane_corr, const IDMRelativeValues& rel_values,
      double delta_time) const;

  virtual std::shared_ptr<BehaviorModel> Clone() const;
};

inline std::shared_ptr<BehaviorModel> BehaviorIDMClassic::Clone() const {
  std::shared_ptr<BehaviorIDMClassic> model_ptr =
      std::make_shared<BehaviorIDMClassic>(*this);
  return model_ptr;
}

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_IDM_IDM_CLASSIC_HPP_
