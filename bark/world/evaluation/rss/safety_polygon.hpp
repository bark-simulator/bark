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

// TODO: make serializable
struct SafetyPolygon {
  double lat_left_safety_distance{0.};
  double lat_right_safety_distance{0.};
  double lon_safety_distance{0.};
  Polygon polygon;
  AgentId agent_id{0};
  Polygon GetPolygon() const {
    return polygon;
  }
  AgentId GetAgentId() const {
    return agent_id;
  }
};

typedef std::shared_ptr<SafetyPolygon> SafetyPolygonPtr;

inline void ComputeSafetyPolygon(
  SafetyPolygon& safe_poly, const ObservedWorld& observed_world) {
  // 1. copy shape and insert into safe_poly
  auto ego_agent = observed_world.GetEgoAgent();
  auto ego_pose = ego_agent->GetCurrentPosition();
  auto ego_state = ego_agent->GetCurrentState();
  auto theta = ego_state(StateDefinition::THETA_POSITION);

  // 2. transform
  std::vector<Point2d> points = ego_agent->GetPolygonFromState(ego_state).obj_.outer();
  for (auto pt : points) {
    double pt_angle = atan2(
      bg::get<1>(ego_pose) - bg::get<1>(pt),
      bg::get<0>(ego_pose) - bg::get<0>(pt));
    double signed_angle_diff_lat = SignedAngleDiff(theta, pt_angle);
    double signed_angle_diff_lon = SignedAngleDiff(theta - 3.14/2., pt_angle);
    double sgn_lat = signed_angle_diff_lat > 0 ? 1 : -1;
    double sgn_lon = signed_angle_diff_lon > 0 ? 1 : -1;

    // lateral safety distancesa
    double lat_dist = 0;
    lat_dist =  sgn_lat < 0 ? safe_poly.lat_left_safety_distance : safe_poly.lat_right_safety_distance;  // NOLINT
    auto lat_proj_angle = sgn_lat > 0 ? theta - 3.14/2. : theta + 3.14/2.;
    if (sgn_lat < 0) {
      auto x_new = bg::get<0>(pt) + lat_dist*cos(lat_proj_angle);
      auto y_new = bg::get<1>(pt) + lat_dist*sin(lat_proj_angle);
      bg::set<0>(pt, x_new);
      bg::set<1>(pt, y_new);
    }

    // longitudinal back and front safety distance
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
  os << ")";
  return os;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_SAFETY_POLYGON_HPP_
