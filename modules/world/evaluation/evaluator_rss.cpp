// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_rss.hpp"
#include <iostream>

using ::ad::map::point::ENUHeading;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;
using ::ad::physics::ParametricValue;
using ::ad::physics::Speed;
using ::ad::rss::situation::VehicleState;
using ::ad::rss::world::Object;
using ::ad::rss::world::RoadArea;
using ::ad::rss::world::Scene;
using ::ad::rss::world::WorldModel;

namespace bg = boost::geometry;

EgoDynamicsOnRoute::EgoDynamicsOnRoute()
    : ego_speed(0.),
      route_heading(0.),
      route_speed_lat(0.),
      route_speed_lon(0.),
      route_accel_lat(0.),
      route_accel_lon(0.),
      avg_route_accel_lat(0.),
      avg_route_accel_lon(0.) {
  timestamp = 0.;
}

namespace modules {
namespace world {

using geometry::Line;
using geometry::Point2d;
using geometry::Polygon;
using map::MapInterfacePtr;
using models::dynamic::State;
using models::dynamic::Trajectory;
using models::dynamic::StateDefinition::THETA_POSITION;
using models::dynamic::StateDefinition::TIME_POSITION;
using models::dynamic::StateDefinition::VEL_POSITION;
using models::dynamic::StateDefinition::X_POSITION;
using models::dynamic::StateDefinition::Y_POSITION;
using objects::Agent;
using objects::AgentPtr;
using opendrive::XodrLanePtr;

typedef std::map<AgentId, AgentPtr> AgentMap;

class World;
namespace evaluation {

::ad::map::match::Object GetMatchObject(
    const AgentPtr &agent, ::ad::physics::Distance const &match_distance) {
  ::ad::map::match::Object match_object;

  Point2d agent_center = agent->GetCurrentPosition();
  Trajectory agent_execut_traj = agent->GetExecutionTrajectory();
  Polygon agent_shape = agent->GetShape();

  match_object.enuPosition.centerPoint.x =
      ::ad::map::point::ENUCoordinate(bg::get<0>(agent_center));
  match_object.enuPosition.centerPoint.y =
      ::ad::map::point::ENUCoordinate(bg::get<1>(agent_center));
  match_object.enuPosition.centerPoint.z = ::ad::map::point::ENUCoordinate(0);
  match_object.enuPosition.heading = ::ad::map::point::createENUHeading(
      agent_execut_traj(agent_execut_traj.rows() - 1, THETA_POSITION));

  match_object.enuPosition.dimension.length =
      Distance(agent_shape.front_dist_ + agent_shape.rear_dist_);
  match_object.enuPosition.dimension.width =
      Distance(agent_shape.left_dist_ + agent_shape.right_dist_);

  match_object.enuPosition.dimension.height = ::ad::physics::Distance(1);
  match_object.enuPosition.enuReferencePoint =
      ::ad::map::access::getENUReferencePoint();

  ::ad::map::match::AdMapMatching map_matching;
  match_object.mapMatchedBoundingBox = map_matching.getMapMatchedBoundingBox(
      match_object.enuPosition, match_distance, ::ad::physics::Distance(2.));

  return match_object;
}

EvaluationReturn EvaluatorRss::Evaluate(const world::World &world) {
  bool ego_safety_state = true;

  // https://intel.github.io/ad-rss-lib/ad_rss_map_integration/ConstructRSSScenes/

  // WorldModel world_model;
  // Object ego_object;
  // Object other_object;
  // RoadArea road_area;
  // Scene scene;

  // ======== generate ego object ========
  double timestamp = world.GetWorldTime();

  MapInterfacePtr map = world.GetMap();
  AgentPtr ego_agent = world.GetAgent(agent_id_);
  AgentMap other_agents = world.GetAgents();

  State ego_state = ego_agent->GetCurrentState();
  Point2d ego_center = ego_agent->GetCurrentPosition();

  XodrLanePtr ego_lane = map->FindXodrLane(ego_center);

  Line lane_ref_line = ego_lane->GetLine();
  Trajectory ego_plan_traj = ego_agent->GetBehaviorTrajectory();
  Trajectory ego_execut_traj = ego_agent->GetExecutionTrajectory();

  ::ad::map::match::Object match_object =
      GetMatchObject(ego_agent, ::ad::physics::Distance(2.0));

  // ======== find the route of ego ========
  // map->FindCurrentRoad()
  // map->GetRoadgraph();
  map::LaneId ego_current_lane_id = map->FindCurrentLane(ego_center);
  map::RoadCorridorPtr ego_road_corridor = ego_agent->GetRoadCorridor();
  map::LaneCorridorPtr ego_current_lane_corridor =
      ego_road_corridor->GetLaneCorridor(ego_current_lane_id);
  map::LanePtr ego_current_lane =
      ego_current_lane_corridor->GetCurrentLane(ego_center);


  // ======== CalculateEgoDynamicsOnRoute ========

  EgoDynamicsOnRoute new_dynamics;

  new_dynamics.timestamp =
      ego_execut_traj(ego_execut_traj.rows() - 1, TIME_POSITION);

  new_dynamics.ego_center = match_object.enuPosition.centerPoint;

  new_dynamics.ego_heading = match_object.enuPosition.heading;
  new_dynamics.ego_speed =
      ego_execut_traj(ego_execut_traj.rows() - 1, VEL_POSITION);

  float s = GetNearestS(lane_ref_line, ego_center);
  new_dynamics.route_heading =
      ::ad::map::point::ENUHeading(GetTangentAngleAtS(lane_ref_line, s));
  new_dynamics.heading_diff = ::ad::map::point::normalizeENUHeading(
      new_dynamics.route_heading - new_dynamics.ego_heading);

  new_dynamics.route_speed_lon =
      std::fabs(std::cos(static_cast<double>(new_dynamics.heading_diff))) *
      new_dynamics.ego_speed;
  new_dynamics.route_speed_lat =
      std::sin(static_cast<double>(new_dynamics.heading_diff)) *
      new_dynamics.ego_speed;

  Duration const delta_time(
      ego_execut_traj(ego_execut_traj.rows() - 1, TIME_POSITION) -
      ego_execut_traj(0, TIME_POSITION));

  new_dynamics.route_accel_lat = Acceleration(
      (new_dynamics.route_speed_lat - ego_dynamics_on_route.route_speed_lat) /
      delta_time);
  new_dynamics.route_accel_lon = Acceleration(
      (new_dynamics.route_speed_lon - ego_dynamics_on_route.route_speed_lon) /
      delta_time);
  new_dynamics.avg_route_accel_lat =
      ((ego_dynamics_on_route.avg_route_accel_lat * 2.) +
       new_dynamics.route_accel_lat) /
      3.;
  new_dynamics.avg_route_accel_lon =
      ((ego_dynamics_on_route.avg_route_accel_lon * 2.) +
       new_dynamics.route_accel_lon) /
      3.;

  new_dynamics.ego_center_within_route = true;
  new_dynamics.crossing_border = false;

  ad::rss::map::RssObjectConversion object_conversion(
      ::ad::rss::world::ObjectId(0u), ::ad::rss::world::ObjectType::EgoVehicle,
      match_object, new_dynamics.ego_speed, default_dynamics_);
  object_conversion.calculateMinStoppingDistance(
      new_dynamics.min_stopping_distance);

  std::cout << new_dynamics.ego_speed << " " << new_dynamics.route_speed_lat
            << " " << new_dynamics.route_speed_lon << " "
            << new_dynamics.route_accel_lat << " "
            << new_dynamics.route_accel_lon << " "
            << new_dynamics.avg_route_accel_lat << " "
            << new_dynamics.avg_route_accel_lon << " "
            << new_dynamics.min_stopping_distance << " " << std::endl;

  ego_dynamics_on_route = new_dynamics;

  // ======== CreateWorldModel ========

  std::vector<AgentPtr> relevent_agents;

  double const relevant_distance = std::max(
      static_cast<double>(ego_dynamics_on_route.min_stopping_distance), 100.);

  for (const auto &other_agent : other_agents) {
    if (geometry::Distance(ego_center,
                           other_agent.second->GetCurrentPosition()) <
        relevant_distance) {
      relevent_agents.push_back(other_agent.second);
    }
  }

  ::ad::rss::map::RssSceneCreation scene_creation(timestamp, default_dynamics_);

  // RssObjectChecker

  // for (auto const vehicle : other_vehicles) {
  //   auto checker = RssObjectChecker(*this, scene_creation, carla_ego_vehicle,
  //   carla_rss_state, green_traffic_lights); checker(vehicle);
  // }

  for (const auto &relevent_agent : relevent_agents) {
    auto const other_match_object =
        GetMatchObject(relevent_agent, ::ad::physics::Distance(2.0));

    State relevent_agent_state = relevent_agent->GetCurrentState();

    Speed relevent_agent_speed = relevent_agent_state(VEL_POSITION);

    // TODO: ego_route, green_traffic_lights

    // _scene_creation.appendScenes(
    //     ::ad::rss::world::ObjectId(agent_id_), ego_match_object,
    //     ego_dynamics_on_route.ego_speed, default_dynamics_,
    //     _carla_rss_state.ego_route,
    //     ::ad::rss::world::ObjectId(relevent_agent->GetAgentId->GetId()),
    //     ::ad::rss::world::ObjectType::OtherVehicle, other_match_object,
    //     relevent_agent_speed, default_dynamics_,
    //     ::ad::rss::map::RssSceneCreation::RestrictSpeedLimitMode::IncreasedSpeedLimit10,
    //     _green_traffic_lights);
  }

  return ego_safety_state;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules