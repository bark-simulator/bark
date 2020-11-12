
// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_
#define BARK_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_

#include "bark/commons/transformation/frenet.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"


namespace bark {
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
  FrenetState(const bark::models::dynamic::State& state,
              const bark::geometry::Line& path);

  bool Valid() const { return FrenetPosition::Valid() && angle <= bark::geometry::B_PI; }

  double vlon;
  double vlat;
  double angle;
};

bark::models::dynamic::State FrenetStateToDynamicState(
    const FrenetState& frenet_state, const bark::geometry::Line& path);

auto ShapeExtensionAtTangentAngle(const double& tangent_angle, const bark::geometry::Polygon& polygon);

FrenetState FrenetStateDiffShapeExtension(const FrenetState& frenet_state1, const bark::geometry::Polygon& polygon1,
                                        const FrenetState& frenet_state2, const bark::geometry::Polygon& polygon2);

}  // namespace transformation
}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_