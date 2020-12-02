// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_
#define BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_

#include <limits>
#include <memory>
#include <string>

#include "bark/world/world.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/world/world.hpp"


namespace bark {
namespace world {
namespace evaluation {

using geometry::Polygon;
using bark::world::AgentId;
using objects::StateDefinition;
using bark::geometry::SignedAngleDiff;
namespace bg = boost::geometry;

/**
 * @brief  SafetyPolygon that contains the information extracted
 *         from the RSS interface.
 * @note   In particular, the longitudinal and lateral safety distances,
 *         the polygon itself, the AgentId it is associtated with.
 * @retval None
 */
struct SafetyPolygon {
  double lat_left_safety_distance{0.};  // lateral left safety distance
  double lat_right_safety_distance{0.};  // lateral right safety distance
  double lon_safety_distance{0.};  // longitudinal safety distance
  Polygon polygon;  // BARK Polygon representing the safety area
  AgentId agent_id{0};  // AgentId of the other agent
  double curr_distance{0.};  // current longitudinal distance to other agent
  Polygon GetPolygon() const {
    return polygon;
  }
  AgentId GetAgentId() const {
    return agent_id;
  }
};

typedef std::shared_ptr<SafetyPolygon> SafetyPolygonPtr;

/**
 * @brief  A function that computes the actual polygon within the SafetyPolygon
 * @note   It uses the current vehicle shape and extends it with the given
 *         safety distances. The longitudinal safety distance is drawn
 *         conditional on whether the other vehicle is preceeding or not.
 *         Based on the scalar values in the SafetyPolygon, it computes and
 *         fills the BARK polygon within the SafetyPolygon struct.
 * @param  safe_poly: SafetyPolgon 
 * @param  observed_world: ObservedWorld of the Agent to obtain geospatial info
 * @retval None
 */
inline void ComputeSafetyPolygon(
  SafetyPolygon& safe_poly, const ObservedWorld& observed_world) {
  // 1. Get the ego agent required values
  auto ego_agent = observed_world.GetEgoAgent();
  auto ego_pose = ego_agent->GetCurrentPosition();
  auto ego_state = ego_agent->GetCurrentState();
  auto theta = ego_state(StateDefinition::THETA_POSITION);

  // 2. transform the polygon of the ego agent so it includes safety distances
  std::vector<Point2d> points =
    ego_agent->GetPolygonFromState(ego_state).obj_.outer();
  for (auto pt : points) {
    // angle to the point in the polygon in respect to the ego pose
    double pt_angle = atan2(
      bg::get<1>(ego_pose) - bg::get<1>(pt),
      bg::get<0>(ego_pose) - bg::get<0>(pt));
    // if the point is on the left side it is [-pi, 0]
    // if the point is on the right side it is [0, pi]
    double signed_angle_diff_lat = SignedAngleDiff(theta, pt_angle);
    // here we tilt the vehicle angle pi/2 to the left so that is is
    // perpendicular to the vehicle
    // pt_angle smaller than zero are behing, otherwise in front
    double signed_angle_diff_lon = SignedAngleDiff(theta - M_PI_2, pt_angle);
    double sgn_lat = signed_angle_diff_lat > 0 ? 1 : -1;
    double sgn_lon = signed_angle_diff_lon > 0 ? 1 : -1;

    // compute lateral safety distances
    double lat_dist = 0;
    lat_dist =  sgn_lat < 0 ? safe_poly.lat_left_safety_distance : safe_poly.lat_right_safety_distance;  // NOLINT
    auto lat_proj_angle = sgn_lat > 0 ? theta - M_PI_2 : theta + M_PI_2;
    // NOTE: often the safety distance is returned as 1+e9
    if (lat_dist < 10000) {
      auto x_new = bg::get<0>(pt) + lat_dist*cos(lat_proj_angle);
      auto y_new = bg::get<1>(pt) + lat_dist*sin(lat_proj_angle);
      bg::set<0>(pt, x_new);
      bg::set<1>(pt, y_new);
    }

    // calculate if the agent is in front or behind
    auto other_agent = observed_world.GetAgents()[safe_poly.agent_id];
    auto other_pose = other_agent->GetCurrentPosition();
    double relative_angle = atan2(
      bg::get<1>(ego_pose) - bg::get<1>(other_pose),
      bg::get<0>(ego_pose) - bg::get<0>(other_pose));
    double diff_angle = SignedAngleDiff(theta - M_PI_2, relative_angle);
    double sgn_lon_in_front = diff_angle > 0 ? 1 : -1;

    // this enables directional computation for the longitudinal
    // safety distance
    // if the signs are different, sgn_lon is set to zero
    // Thus, the original point of the polygon will not be modified
    // NOTE: often the safety distance is returned as 1+e9
    if (sgn_lon_in_front*sgn_lon < 0. ||
        safe_poly.lon_safety_distance > 10000.)
      sgn_lon = 0.;

    // longitudinal safety distance
    bg::set<0>(
      pt, bg::get<0>(pt) + sgn_lon*safe_poly.lon_safety_distance*cos(theta));
    bg::set<1>(
      pt, bg::get<1>(pt) + sgn_lon*safe_poly.lon_safety_distance*sin(theta));
    safe_poly.polygon.AddPoint(pt);
  }
}

inline std::ostream &operator<<(std::ostream &os, const SafetyPolygon& v)
{
  os << "SafetyPolygon(";
  os << "lat_left_safety_distance:";
  os << v.lat_left_safety_distance;
  os << ",";
  os << "lat_right_safety_distance:";
  os << v.lat_right_safety_distance;
  os << ",";
  os << "lon_safety_distance:";
  os << v.lon_safety_distance;
  os << ",";
  os << "agent_id:";
  os << v.agent_id;
  os << ",";
  os << "curr_distance:";
  os << v.curr_distance;
  os << ")";
  return os;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_
