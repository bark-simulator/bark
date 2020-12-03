// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_COMMONS_HPP_
#define BARK_WORLD_EVALUATION_COMMONS_HPP_

#include <limits>
#include <memory>
#include <string>

#include "bark/world/world.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/commons/transformation/frenet.hpp"
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/world/evaluation/rss/safety_polygon.hpp"

#include <ad/rss/situation/Physics.hpp>
#include <ad/rss/situation/RssFormulas.hpp>

namespace bark {
namespace world {
namespace evaluation {

using geometry::Polygon;
using ::ad::rss::situation::calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern;
using ::ad::rss::situation::calculateLateralDistanceOffsetAfterStatedBrakingPattern;
using ad::physics::Distance;
using ad::physics::Acceleration;
using ad::physics::Duration;
using bark::commons::transformation::FrenetState;

/**
 * @brief  Calculates the min. braking distance for an agent
 * @note   
 * @param  observed_world: ObservedWorld for which the braking distance
 *                         should be dran
 * @param  rss_params:  RSSDynamics either for ego ot others
 * @retval SafetyPolygon
 */
inline SafetyPolygon ComputeMinBrakingPolygon(
  const ObservedWorld& observed_world, const commons::ParamsPtr& rss_params) {
  auto agent = observed_world.GetEgoAgent();
  auto state = agent->GetCurrentState();
  auto lane_corr = observed_world.GetLaneCorridor();
  auto center_line = lane_corr->GetCenterLine();

  // calculate vlon and vlat
  FrenetState frenet_state(state, center_line);

  // parameters
  Distance min_stopping_distance_lon;
  Acceleration accel_max_lon = Acceleration(
    rss_params->GetReal("BrakeLonMax", "maximum deceleration", -1.7));
  Acceleration brake_min_correct_lon = Acceleration(rss_params->GetReal(
    "BrakeLonMinCorrect", "minimum deceleration of oncoming vehicle", -1.67));
  Duration response_time = Duration(
    rss_params->GetReal("TimeResponse", "response time of the ego vehicle", 0.2));
  Distance min_stopping_distance_lat;
  Acceleration accel_max_lat = Acceleration(
    rss_params->GetReal("AccLatBrakeMax", "maximum lateral acceleration", 0.2));
  Acceleration brake_min_correct_lat = Acceleration(
    rss_params->GetReal("AccLatBrakeMin", "minimum lateral braking", -0.8));

  // longitudinal min braking distance
  auto const lon_result = calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern(
    frenet_state.vlon,
    ad::physics::Speed::getMax(),
    response_time,
    accel_max_lon,
    brake_min_correct_lon,
    min_stopping_distance_lon);
  
  // lateral min braking distance
  auto const lat_result = calculateLateralDistanceOffsetAfterStatedBrakingPattern(
    frenet_state.vlat,
    response_time,
    accel_max_lat,
    brake_min_correct_lat,
    min_stopping_distance_lat);
  
  // fill longitdunial and lateral values
  SafetyPolygon safety_poly;

  // if the distances could not be calculated
  if (!lon_result || !lat_result)
    return safety_poly;
  
  safety_poly.lat_left_safety_distance = min_stopping_distance_lat;
  safety_poly.lat_right_safety_distance = min_stopping_distance_lat;
  safety_poly.lon_safety_distance = min_stopping_distance_lon;

  // compute actual BARK polygon
  ComputeSafetyPolygon(
    safety_poly, observed_world, LonDirectionMode::FRONT);

  return safety_poly;
} 


}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_COMMONS_HPP_
