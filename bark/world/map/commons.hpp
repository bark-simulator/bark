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
    auto curr_lane_corr = observed_world.GetLaneCorridor();
    auto center_line = curr_lane_corr->GetCenterLine();
    FrenetState frenet_state(ego_state, center_line);

    // if any point of the vehicle shape is within the target corr
    for (auto pt : vehicle_shape.obj_.outer()) {
      if(Within(pt, target_corr->GetMergedPolygon())) {
        return target_corr;
      }
    }

    // if the deviation is too large
    double deviation_angle = 0.2;
    if (fabs(frenet_state.angle) < deviation_angle) {
      return curr_lane_corr;
    } else {
      return target_corr;
    }
}

}  // namespace map
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_MAP_COMMONS_HPP_
