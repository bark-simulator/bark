// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#ifndef BARK_WORLD_MAP_COMMONS_HPP_
#define BARK_WORLD_MAP_COMMONS_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "bark/geometry/geometry.hpp"
#include "bark/world/world.hpp"
#include "bark/commons/transformation/frenet.hpp"
#include "bark/commons/transformation/frenet_state.hpp"


namespace bark {
namespace world {
namespace map {

using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Within;
using bark::world::ObservedWorld;
namespace bg = boost::geometry;
using models::dynamic::StateDefinition;
using bark::geometry::SignedAngleDiff;
using commons::transformation::FrenetState;

/**
 * @brief  Function that selects LaneCorridors
 * @note   If an agent is in two LaneCorridors with its shape
 *         the other one it selected if the front points of the
 *         agent are within the other lane corridor.
 *         This models an entering of the other lane corridor.
 * @param  observed_world: ObservedWorld of agent
 * @param  target_corr: fallback corridor in case calculation fails
 * @retval LaneCorridorPtr: the computed lane corridor
 */
inline LaneCorridorPtr ChooseLaneCorridorBasedOnVehicleState(
  const ObservedWorld& observed_world, const LaneCorridorPtr& target_corr) {
    // ego info
    auto ego_agent = observed_world.GetEgoAgent();
    auto ego_pose = ego_agent->GetCurrentPosition();
    auto ego_state = ego_agent->GetCurrentState();
    auto theta = ego_state(StateDefinition::THETA_POSITION); 
    Polygon vehicle_shape = ego_agent->GetPolygonFromState(ego_state);

    // road and lane corridor info
    auto road_corridor = ego_agent->GetRoadCorridor();

    // 1. check if vehicle shape intersects with an other lane corridor
    LaneCorridorPtr other_lane_corr;
    for (auto& lc : road_corridor->GetUniqueLaneCorridors()) {
      if (Collide(vehicle_shape, lc->GetMergedPolygon()) &&
          lc != target_corr)
        other_lane_corr = lc;
    }

    auto curr_lane_corr = observed_world.GetLaneCorridor();
    auto center_line = curr_lane_corr->GetCenterLine();
    FrenetState frenet_state(ego_state, center_line);

    // auto deviation_angle = observed_world.GetParams()->GetReal(
    //   "DeviationAngleCenterLine",
    //   "angle deviation threshold form which the target_corr on is returned",
    //   0.15);
    double deviation_angle = 0.1;
    // target_corr should always be the left one in the merging map
    // if we are standing leaning on the right LaneCorridor
    // we want to use the left corridor
    // in the case we are on the target corridor, we still will return it
    if (fabs(frenet_state.angle) > deviation_angle)
      return target_corr;
    
    // if it is only in one lane corridor
    if (other_lane_corr == nullptr) {
      // in this case we are entirely in one lane corridor
      // here we want to only account for the preceeding vehicle
      return curr_lane_corr;
    }


    // check if the front points of the ego vehicle are in the
    // other lane corridor
    for (auto pt : vehicle_shape.obj_.outer()) {
      double pt_angle = atan2(
        bg::get<1>(ego_pose) - bg::get<1>(pt),
        bg::get<0>(ego_pose) - bg::get<0>(pt));
      double signed_angle_diff_lon = SignedAngleDiff(theta - M_PI_2, pt_angle);
      // if a point in the front of the polygon is inside the other lane corridor
      // return that lane corridor
      if(Within(pt, other_lane_corr->GetMergedPolygon()) &&
         signed_angle_diff_lon > 0) {
        return other_lane_corr;
      }
    }

    // 3. fallback return and log message
    // VLOG(4) << "Could not calculate the lane corridor." << std::endl;
    return target_corr;
}

}  // namespace map
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_MAP_COMMONS_HPP_
