
// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_
#define MODULES_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_

#include "modules/commons/transformation/frenet.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/geometry/line.hpp"

namespace modules {
namespace commons {
namespace transformation {

struct FrenetState : public FrenetPosition {
  FrenetState() : FrenetPosition() {}
  FrenetState(const double& longitudinal, const double& lateral,
              const double& vlongitudinal, const double& vlateral,
              const double& angle)
      : FrenetPosition(longitudinal, lateral),
        vlon(vlongitudinal),
        vlat(vlateral),
        angle(angle) {}
  FrenetState(const modules::models::dynamic::State& state,
              const modules::geometry::Line& path);

  double vlon;
  double vlat;
  double angle;
};

modules::models::dynamic::State FrenetStateToDynamicState(
    const FrenetState& frenet_state, const modules::geometry::Line& path);

}  // namespace transformation
}  // namespace commons
}  // namespace modules

#endif  // MODULES_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_