
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
  FrenetState() : FrenetPosition(),
                  vlon(std::numeric_limits<double>::max()),
                  vlat(std::numeric_limits<double>::max()),
                  angle(std::numeric_limits<double>::max()) {}
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

  bool Valid() const { return FrenetPosition::Valid() && angle <= bark::geometry::B_PI; }

  double vlon;
  double vlat;
  double angle;  // in fact, this is theta_road - theta_vehicle
  double angleRoad;
};

struct FrenetStateDifference : public FrenetState {
  // Represents differences between frenet state and also considers projected shapes based on tangent angles
  // Conventions: 
  // - positive longitudinal difference if frenet_state2 is in "front" of frenet_state1
  // - positive lateral difference if frenet_state2 is in "left" of frenet_state1
  // - positive long and velocities diff if frenet state 2 is "faster" than frenet state1
  // - positive angle difference if frenet state2 is turned more left than frenet state 1
  // - boolean flags lat_zeroed and lon_zeroed indicate if the lateral/longitudinal shape-based distance is zero
  //                  giving that the calculated distances actually represent a state-based distance
  FrenetStateDifference() : FrenetState(), from(), to(), lat_zeroed(false), lon_zeroed(false) {}
  FrenetStateDifference(const FrenetState& frenet_state1, const bark::geometry::Polygon& polygon1,
                                        const FrenetState& frenet_state2, const bark::geometry::Polygon& polygon2);
  FrenetState from;
  FrenetState to;
  bool lat_zeroed;
  bool lon_zeroed;
};

bark::models::dynamic::State FrenetStateToDynamicState(
    const FrenetState& frenet_state, const bark::geometry::Line& path);

typedef struct{double front_dist; double rear_dist; double left_dist; double right_dist;} ShapeExtension;
ShapeExtension ShapeExtensionAtTangentAngle(const double& tangent_angle, const bark::geometry::Polygon& polygon);

double LatAccStreetToVehicleCs(
    double acc_lat_street, double acc_lon, double delta_time,
    const bark::models::dynamic::State& current_state,
    const FrenetState& current_frenet_state,
    const FrenetState& last_frenet_state);

}  // namespace transformation
}  // namespace commons
}  // namespace bark

#endif  // BARK_COMMONS_TRANSFORMATION_FRENET_STATE_HPP_