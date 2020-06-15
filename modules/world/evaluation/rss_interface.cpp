// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/rss_interface.hpp"

using ::ad::map::point::ENUCoordinate;
using ::ad::map::route::FullRoute;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;

namespace modules {
namespace world {
namespace evaluation {

bool RssInterface::initializeOpenDriveMap(
    const std::string &opendrive_file_name) {
  std::ifstream opendrive_file(opendrive_file_name);
  std::string opendrive_file_content =
      std::string{std::istreambuf_iterator<char>(opendrive_file),
                  std::istreambuf_iterator<char>()};

  ::ad::map::access::cleanup();
  // TODO(chan): define IntersectionType
  // 2nd argument: the value to reduce overlapping between two lanes they are
  // intersected
  bool result = ::ad::map::access::initFromOpenDriveContent(
      opendrive_file_content, 0,
      ::ad::map::intersection::IntersectionType::TrafficLight,
      ::ad::map::landmark::TrafficLightType::UNKNOWN);

  if (result == false) {
    LOG(ERROR) << "Failed to initialize from OpenDrive map : "
               << opendrive_file_content << std::endl;
  }

  return result;
}

::ad::rss::world::RssDynamics RssInterface::GenerateVehicleDynamics(
    double lon_max_accel, double lon_max_brake, double lon_min_brake,
    double lon_min_brake_correct, double lat_max_accel, double lat_min_brake,
    double lat_fluctuation_margin, double response_time) {
  ::ad::rss::world::RssDynamics dynamics;
  dynamics.alphaLon.accelMax = Acceleration(lon_max_accel);
  dynamics.alphaLon.brakeMax = Acceleration(lon_max_brake);
  dynamics.alphaLon.brakeMin = Acceleration(lon_min_brake);
  dynamics.alphaLon.brakeMinCorrect = Acceleration(lon_min_brake_correct);
  dynamics.alphaLat.accelMax = Acceleration(lat_max_accel);
  dynamics.alphaLat.brakeMin = Acceleration(lat_min_brake);
  dynamics.lateralFluctuationMargin = Distance(lat_fluctuation_margin);
  dynamics.responseTime = Duration(response_time);

  return dynamics;
}

::ad::rss::world::RssDynamics RssInterface::GenerateDefaultVehicleDynamics() {
  return GenerateVehicleDynamics(3.5, -8., -4., -3., 0.2, -0.8, 0.1, 1.);
}

::ad::map::match::Object RssInterface::GetMatchObject(
    const AgentPtr &agent, const Distance &match_distance) {
  ::ad::map::match::Object object;

  Point2d agent_center = agent->GetCurrentPosition();
  Trajectory agent_execut_traj = agent->GetExecutionTrajectory();
  Polygon agent_shape = agent->GetShape();

  object.enuPosition.centerPoint.x =
      ENUCoordinate(static_cast<double>(bg::get<0>(agent_center)));
  object.enuPosition.centerPoint.y =
      ENUCoordinate(static_cast<double>(bg::get<1>(agent_center)));
  object.enuPosition.centerPoint.z = ENUCoordinate(0);
  object.enuPosition.heading =
      ::ad::map::point::createENUHeading(static_cast<double>(
          agent_execut_traj(agent_execut_traj.rows() - 1, THETA_POSITION)));

  object.enuPosition.dimension.length = static_cast<double>(
      Distance(agent_shape.front_dist_ + agent_shape.rear_dist_));
  object.enuPosition.dimension.width = static_cast<double>(
      Distance(agent_shape.left_dist_ + agent_shape.right_dist_));
  object.enuPosition.dimension.height = Distance(1.5);
  object.enuPosition.enuReferencePoint =
      ::ad::map::access::getENUReferencePoint();

  ::ad::map::match::AdMapMatching map_matching;
  object.mapMatchedBoundingBox = map_matching.getMapMatchedBoundingBox(
      object.enuPosition, match_distance, Distance(2.));

  return object;
}

FullRoute RssInterface::GenerateRoute(
    const world::World &world, const AgentId &agent_id,
    const ::ad::map::match::Object &matched_object) {
  AgentPtr agent = world.GetAgent(agent_id);
  Point2d agent_center = agent->GetCurrentPosition();
  map::LaneId agent_lane_id = world.GetMap()->FindCurrentLane(agent_center);
  map::RoadCorridorPtr agent_road_corridor = agent->GetRoadCorridor();
  map::LaneCorridorPtr agent_lane_corridor =
      agent_road_corridor->GetLaneCorridor(agent_lane_id);

  geometry::Line agent_lane_center_line = agent_lane_corridor->GetCenterLine();

  float s_start = GetNearestS(agent_lane_center_line, agent_center);
  float s_end = GetNearestS(
      agent_lane_center_line,
      agent_lane_center_line.obj_.at(agent_lane_center_line.obj_.size() - 1));

  std::vector<::ad::map::point::ENUPoint> routing_targets;

  while (s_start <= s_end) {
    geometry::Point2d traj_point = GetPointAtS(agent_lane_center_line, s_start);
    routing_targets.push_back(::ad::map::point::createENUPoint(
        bg::get<0>(traj_point), bg::get<1>(traj_point), 0));
    s_start += 2;
  }

  std::vector<FullRoute> routes;
  std::vector<double> routes_probability;

  for (const auto &position :
       matched_object.mapMatchedBoundingBox.referencePointPositions[int32_t(
           ::ad::map::match::ObjectReferencePoints::Center)]) {
    auto starting_point = position.lanePoint.paraPoint;
    auto projected_starting_point = starting_point;

    if (!::ad::map::lane::isHeadingInLaneDirection(
            starting_point, matched_object.enuPosition.heading)) {
      ::ad::map::lane::projectPositionToLaneInHeadingDirection(
          starting_point, matched_object.enuPosition.heading,
          projected_starting_point);
    }

    auto route_starting_point = ::ad::map::route::planning::createRoutingPoint(
        projected_starting_point, matched_object.enuPosition.heading);

    if (!routing_targets.empty() &&
        ::ad::map::point::isValid(routing_targets)) {
      FullRoute route = ::ad::map::route::planning::planRoute(
          route_starting_point, routing_targets,
          ::ad::map::route::RouteCreationMode::AllRoutableLanes);
      routes.push_back(route);
      routes_probability.push_back(position.probability);
    } else {
      std::vector<FullRoute> possible_routes =
          ::ad::map::route::planning::predictRoutesOnDistance(
              route_starting_point, Distance(50.),
              ::ad::map::route::RouteCreationMode::AllRoutableLanes);
      for (const auto &possible_route : possible_routes) {
        routes.push_back(possible_route);
        routes_probability.push_back(0);
      }
    }
  }

  FullRoute final_route;

  if (routes.empty()) {
    LOG(ERROR) << "Could not find any route to the targets" << std::endl;
  } else {
    int best_route_idx = std::distance(
        routes_probability.begin(),
        std::max_element(routes_probability.begin(), routes_probability.end()));
    final_route = routes[best_route_idx];
  }
  return final_route;
}

AgentState RssInterface::CalculateExecutionState(
    const Trajectory &execution_trajectory,
    const ::ad::rss::world::RssDynamics &agent_dynamics) {
  AgentState execution_state;

  int last_state_idx = execution_trajectory.rows() - 1;

  execution_state.timestamp =
      execution_trajectory(last_state_idx, TIME_POSITION);
  execution_state.center = ::ad::map::point::createENUPoint(
      static_cast<double>(execution_trajectory(last_state_idx, X_POSITION)),
      static_cast<double>(execution_trajectory(last_state_idx, Y_POSITION)),
      0.);
  execution_state.heading =
      ::ad::map::point::createENUHeading(static_cast<double>(
          execution_trajectory(last_state_idx, THETA_POSITION)));
  execution_state.speed = Speed(
      static_cast<double>(execution_trajectory(last_state_idx, VEL_POSITION)));
  execution_state.min_stopping_distance =
      CalculateMinStoppingDistance(execution_state.speed, agent_dynamics);

  return execution_state;
}

Distance RssInterface::CalculateMinStoppingDistance(
    const Speed &speed, const ::ad::rss::world::RssDynamics &agent_dynamics) {
  Distance minStoppingDistance;
  Speed MaxSpeedAfterResponseTime;

  auto result = ::ad::rss::situation::calculateSpeedAfterResponseTime(
      ::ad::rss::situation::CoordinateSystemAxis::Longitudinal,
      std::fabs(speed), Speed::getMax(), agent_dynamics.alphaLon.accelMax,
      agent_dynamics.responseTime, MaxSpeedAfterResponseTime);

  result = result &&
           ::ad::rss::situation::calculateStoppingDistance(
               MaxSpeedAfterResponseTime,
               agent_dynamics.alphaLon.brakeMinCorrect, minStoppingDistance);

  minStoppingDistance +=
      MaxSpeedAfterResponseTime * agent_dynamics.responseTime;

  if (result == false) {
    LOG(ERROR)
        << "Failed to calculate maximum possible speed after response time "
        << agent_dynamics.responseTime << std::endl;
  }

  return minStoppingDistance;
}

::ad::rss::world::WorldModel RssInterface::CreateWorldModel(
    const world::World &world, const AgentId &ego_id,
    const AgentState &ego_state,
    const ::ad::map::match::Object &ego_matched_object,
    const ::ad::rss::world::RssDynamics &ego_dynamics,
    const ::ad::map::route::FullRoute &ego_route) {
  std::vector<AgentPtr> relevent_agents;
  double const relevant_distance =
      static_cast<double>(ego_state.min_stopping_distance);

  geometry::Point2d ego_center(ego_state.center.x, ego_state.center.y);
  AgentMap other_agents = world.GetAgents();

  for (const auto &other_agent : other_agents) {
    if (other_agent.second->GetAgentId() != ego_id) {
      if (geometry::Distance(ego_center,
                             other_agent.second->GetCurrentPosition()) <
          relevant_distance) {
        relevent_agents.push_back(other_agent.second);
      }
    }
  }

  ::ad::rss::map::RssSceneCreation scene_creation(ego_state.timestamp,
                                                  ego_dynamics);
  ::ad::map::landmark::LandmarkIdSet
      green_traffic_lights;  // we don't care about traffic lights right now

  for (const auto &relevent_agent : relevent_agents) {
    auto const other_matched_object =
        GetMatchObject(relevent_agent, Distance(2.0));
    models::dynamic::State relevent_agent_state =
        relevent_agent->GetCurrentState();
    Speed relevent_agent_speed = relevent_agent_state(VEL_POSITION);

    ::ad::rss::world::RssDynamics relevent_agent_dynamics =
        GenerateDefaultVehicleDynamics();

    scene_creation.appendScenes(
        ::ad::rss::world::ObjectId(ego_id), ego_matched_object, ego_state.speed,
        ego_dynamics, ego_route,
        ::ad::rss::world::ObjectId(relevent_agent->GetAgentId()),
        ::ad::rss::world::ObjectType::OtherVehicle, other_matched_object,
        relevent_agent_speed, relevent_agent_dynamics,
        ::ad::rss::map::RssSceneCreation::RestrictSpeedLimitMode::
            IncreasedSpeedLimit10,
        green_traffic_lights);
  }

  return scene_creation.getWorldModel();
}

bool RssInterface::RssCheck(::ad::rss::world::WorldModel world_model) {
  ::ad::rss::core::RssCheck rss_check;
  ::ad::rss::situation::SituationSnapshot situation_snapshot;
  ::ad::rss::state::RssStateSnapshot rss_state_snapshot;
  ::ad::rss::state::ProperResponse proper_response;
  ::ad::rss::world::AccelerationRestriction acceleration_restriction;

  bool result = rss_check.calculateAccelerationRestriction(
      world_model, situation_snapshot, rss_state_snapshot, proper_response,
      acceleration_restriction);

  // std::map<AgentId, bool> relevent_agents_safety_check_result;

  bool is_agent_safe = true;

  if (result == true) {
    for (auto const state : rss_state_snapshot.individualResponses) {
      is_agent_safe = is_agent_safe && !::ad::rss::state::isDangerous(state);
    }
  } else {
    LOG(ERROR) << "Failed to perform RSS check" << std::endl;
  }
  return is_agent_safe;
}

bool RssInterface::IsAgentSafe(const World &world, const AgentId &agent_id) {
  AgentPtr agent = world.GetAgent(agent_id);

  ::ad::map::match::Object matched_object =
      GetMatchObject(agent, Distance(2.0));
  ::ad::map::route::FullRoute agent_route =
      GenerateRoute(world, agent_id, matched_object);
  ::ad::rss::world::RssDynamics agent_dynamics =
      GenerateDefaultVehicleDynamics();
  Trajectory agent_execut_traj = agent->GetExecutionTrajectory();
  AgentState agent_state =
      CalculateExecutionState(agent_execut_traj, agent_dynamics);

  ::ad::rss::world::WorldModel rss_world_model =
      CreateWorldModel(world, agent_id, agent_state, matched_object,
                       agent_dynamics, agent_route);

  return RssCheck(rss_world_model);
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules