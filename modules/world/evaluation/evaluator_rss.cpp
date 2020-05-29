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
      ::ad::map::point::ENUCoordinate(static_cast<double>(bg::get<0>(agent_center)));
  match_object.enuPosition.centerPoint.y =
      ::ad::map::point::ENUCoordinate(static_cast<double>(bg::get<1>(agent_center)));
  match_object.enuPosition.centerPoint.z = ::ad::map::point::ENUCoordinate(0);
  match_object.enuPosition.heading = ::ad::map::point::createENUHeading(
      static_cast<double>(agent_execut_traj(agent_execut_traj.rows() - 1, THETA_POSITION)));
  // std::cout << bg::get<0>(agent_center) << " " <<bg::get<1>(agent_center)<<std::endl;

  match_object.enuPosition.dimension.length =
      static_cast<double>(Distance(agent_shape.front_dist_ + agent_shape.rear_dist_));
  match_object.enuPosition.dimension.width =
      static_cast<double>(Distance(agent_shape.left_dist_ + agent_shape.right_dist_));
  match_object.enuPosition.dimension.height = ::ad::physics::Distance(1.5);
  match_object.enuPosition.enuReferencePoint =
      ::ad::map::access::getENUReferencePoint();

  ::ad::map::match::AdMapMatching map_matching;
  match_object.mapMatchedBoundingBox = map_matching.getMapMatchedBoundingBox(
      match_object.enuPosition, match_distance, ::ad::physics::Distance(2.));
  // std::cout << "Match Object: "<< match_object << std::endl;
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

  ::ad::map::match::Object ego_match_object =
      GetMatchObject(ego_agent, ::ad::physics::Distance(2.0));

  // std::cout << "Ego match object: " <<::ad::map::point::isValid(ego_match_object.enuPosition.centerPoint, true) << std::endl;

  // ======== UpdateRoute ========

  // find the points on the route of ego 
  // map->FindCurrentRoad()
  // map->GetRoadgraph();
  map::LaneId ego_lane_id = map->FindCurrentLane(ego_center);
  map::RoadCorridorPtr ego_road_corridor = ego_agent->GetRoadCorridor();
  map::LaneCorridorPtr ego_lane_corridor =
      ego_road_corridor->GetLaneCorridor(ego_lane_id);
  // map::LanePtr ego_lane =
  //     ego_lane_corridor->GetCurrentLane(ego_center);

  geometry::Line ego_lane_center_line = ego_lane_corridor->GetCenterLine();

  float s_start = GetNearestS(ego_lane_center_line, ego_center);
  float s_end = GetNearestS(ego_lane_center_line,ego_lane_center_line.obj_.at(ego_lane_center_line.obj_.size() - 1));

  std::vector<::ad::map::point::ENUPoint> ego_routing_targets;
  while(s_start<=s_end){
    geometry::Point2d traj_point = GetPointAtS(ego_lane_center_line, s_start);
    ego_routing_targets.push_back(::ad::map::point::createENUPoint(bg::get<0>(traj_point),bg::get<1>(traj_point),0));
    s_start+=3;
  }
  
  // create the route
  // TODO: why generate multiple routes?
  ::ad::physics::Distance const route_target_length(50.);
  std::vector<::ad::map::route::FullRoute> all_new_routes;
  for (const auto &position :
        ego_match_object.mapMatchedBoundingBox
            .referencePointPositions[int32_t(::ad::map::match::ObjectReferencePoints::Center)]) {
    auto start_point = position.lanePoint.paraPoint;
    auto projected_start_point = start_point;
    if (!::ad::map::lane::isHeadingInLaneDirection(start_point,ego_match_object.enuPosition.heading)) {
      std::cout<< "EgoVehicle heading in opposite lane direction" << std::endl;
      if (::ad::map::lane::projectPositionToLaneInHeadingDirection(
              start_point, ego_match_object.enuPosition.heading, projected_start_point)) {
        std::cout<< "Projected to lane {}"<<" "<<  projected_start_point.laneId << std::endl;
      }
    }
    std::cout<< "Route start_point: {}, projected_start_point: {}"<<" " << start_point <<" "<< projected_start_point  << std::endl;
    auto routing_start_point = ::ad::map::route::planning::createRoutingPoint(
        projected_start_point, ego_match_object.enuPosition.heading);
    if (!ego_routing_targets.empty() && ::ad::map::point::isValid(ego_routing_targets)) {
      auto new_route = ::ad::map::route::planning::planRoute(routing_start_point, ego_routing_targets,
                                                              ::ad::map::route::RouteCreationMode::AllRoutableLanes);
      all_new_routes.push_back(new_route);
    } else {
      auto new_routes = ::ad::map::route::planning::predictRoutesOnDistance(
          routing_start_point, route_target_length, ::ad::map::route::RouteCreationMode::AllRoutableLanes);

      for (const auto &new_route : new_routes) {
        // extend route with all lanes
        all_new_routes.push_back(new_route);
      }
    }
  }
  ::ad::map::route::FullRoute ego_route = all_new_routes[0];
  // std::cout<<"Ego route: "<< ego_route<<std::endl;
  

  // ======== CalculateEgoDynamicsOnRoute ========

  EgoDynamicsOnRoute new_dynamics;

  new_dynamics.timestamp =
      ego_execut_traj(ego_execut_traj.rows() - 1, TIME_POSITION);

  new_dynamics.ego_center = ego_match_object.enuPosition.centerPoint;

  new_dynamics.ego_heading = ego_match_object.enuPosition.heading;
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
      ego_match_object, new_dynamics.ego_speed, default_dynamics_);
  object_conversion.calculateMinStoppingDistance(
      new_dynamics.min_stopping_distance);

  // std::cout << new_dynamics.ego_speed << " " << new_dynamics.route_speed_lat
  //           << " " << new_dynamics.route_speed_lon << " "
  //           << new_dynamics.route_accel_lat << " "
  //           << new_dynamics.route_accel_lon << " "
  //           << new_dynamics.avg_route_accel_lat << " "
  //           << new_dynamics.avg_route_accel_lon << " "
  //           << new_dynamics.min_stopping_distance << " " << std::endl;

  ego_dynamics_on_route = new_dynamics;

  // ======== CreateWorldModel ========

  std::vector<AgentPtr> relevent_agents;

  double const relevant_distance = std::max(
      static_cast<double>(ego_dynamics_on_route.min_stopping_distance), 100.);

  for (const auto &other_agent : other_agents) {
    if (other_agent.second->GetAgentId()!=agent_id_){
      if (geometry::Distance(ego_center,
                           other_agent.second->GetCurrentPosition()) <
        relevant_distance) {
      relevent_agents.push_back(other_agent.second);
      }
    }
  }

  ::ad::rss::map::RssSceneCreation scene_creation(timestamp, default_dynamics_);

  // RssObjectChecker

  // for (auto const vehicle : other_vehicles) {
  //   auto checker = RssObjectChecker(*this, scene_creation, carla_ego_vehicle,
  //   carla_rss_state, green_traffic_lights); checker(vehicle);
  // }
  ::ad::map::landmark::LandmarkIdSet green_traffic_lights;
  for (const auto &relevent_agent : relevent_agents) {
    auto const other_match_object =
        GetMatchObject(relevent_agent, ::ad::physics::Distance(2.0));

    State relevent_agent_state = relevent_agent->GetCurrentState();

    Speed relevent_agent_speed = relevent_agent_state(VEL_POSITION);

    scene_creation.appendScenes(
        ::ad::rss::world::ObjectId(agent_id_), ego_match_object,
        new_dynamics.ego_speed, default_dynamics_,
        ego_route,
        ::ad::rss::world::ObjectId(relevent_agent->GetAgentId()),
        ::ad::rss::world::ObjectType::OtherVehicle, other_match_object,
        relevent_agent_speed, default_dynamics_,
        ::ad::rss::map::RssSceneCreation::RestrictSpeedLimitMode::IncreasedSpeedLimit10,
        green_traffic_lights);

  }

  ::ad::rss::world::WorldModel world_model=scene_creation.getWorldModel();

  // ======== PerformCheck ========
  ::ad::rss::core::RssCheck rss_check;
  ::ad::rss::situation::SituationSnapshot situation_snapshot;
  ::ad::rss::state::RssStateSnapshot rss_state_snapshot;
  ::ad::rss::state::ProperResponse proper_response;
  ::ad::rss::world::AccelerationRestriction acceleration_restriction;

  bool result = rss_check.calculateAccelerationRestriction(
      world_model, situation_snapshot, rss_state_snapshot,
      proper_response, acceleration_restriction);
  
  std::cout<< "acceleration restrictions: "<< result <<std::endl;

  for (auto const state : rss_state_snapshot.individualResponses) {
    std::cout <<"Dangerous: "<< ::ad::rss::state::isDangerous(state) <<std::endl;
  }

  return ego_safety_state;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules