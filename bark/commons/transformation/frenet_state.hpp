
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
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace commons {
namespace transformation {

struct FrenetState : public FrenetPosition {
  FrenetState() : FrenetPosition() {}
  FrenetState(const double& longitudinal, const double& lateral,
              const double& vlongitudinal, const double& vlateral,
              const double& angle, double angleRoad, double vLatRoad)
      : FrenetPosition(longitudinal, lateral),
        vlon(vlongitudinal),
        vlat(vlateral),
        angle(angle),
        angleRoad(angleRoad) {}
  FrenetState(const bark::models::dynamic::State& state,
              const bark::geometry::Line& path);

  double vlon;
  double vlat;
  double angle;  // in fact, this is theta_road - theta_vehicle
  double angleRoad;
};

bark::models::dynamic::State FrenetStateToDynamicState(
    const FrenetState& frenet_state, const bark::geometry::Line& path);

double LatAccStreetToVehicleCs(
    double acc_lat_street, double acc_lon, double delta_time,
    const bark::models::dynamic::State& current_state,
    const FrenetState& current_frenet_state,
    const FrenetState& last_frenet_state);

}  // namespace transformation
}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_