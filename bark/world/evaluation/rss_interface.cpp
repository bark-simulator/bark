// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/rss_interface.hpp"

using ::ad::map::point::ENUCoordinate;
using ::ad::map::route::FullRoute;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;

namespace bark {
namespace world {
namespace evaluation {

bool RssInterface::initializeOpenDriveMap(
    const std::string &opendrive_file_name) {
  std::ifstream opendrive_file(opendrive_file_name);
  std::string opendrive_file_content =
      std::string{std::istreambuf_iterator<char>(opendrive_file),
                  std::istreambuf_iterator<char>()};
  ::ad::map::access::cleanup();

  // the 2nd argument is the value of narrowing overlapping between two lanes,
  // it is possibly relevent if the map has intersection
  bool result = ::ad::map::access::initFromOpenDriveContent(
      opendrive_file_content, 0.01,
      ::ad::map::intersection::IntersectionType::TrafficLight,
      ::ad::map::landmark::TrafficLightType::UNKNOWN);

  if (result == false)
    LOG(ERROR) << "RSS failed to initialize from OpenDrive map : "
               << opendrive_file_content << std::endl;

  return result;
}

::ad::rss::world::RssDynamics RssInterface::GenerateVehicleDynamicsParameters(
    float lon_max_accel, float lon_max_brake, float lon_min_brake,
    float lon_min_brake_correct, float lat_max_accel, float lat_min_brake,
    float lat_fluctuation_margin, float response_time) {
  ::ad::rss::world::RssDynamics dynamics;

  // RSS dynamics values along longitudinal coordinate system axis
  dynamics.alphaLon.accelMax = Acceleration(lon_max_accel);
  dynamics.alphaLon.brakeMax = Acceleration(lon_max_brake);
  dynamics.alphaLon.brakeMin = Acceleration(lon_min_brake);
  dynamics.alphaLon.brakeMinCorrect = Acceleration(lon_min_brake_correct);

  // RSS dynamics values along lateral coordinate system axis
  dynamics.alphaLat.accelMax = Acceleration(lat_max_accel);
  dynamics.alphaLat.brakeMin = Acceleration(lat_min_brake);
  dynamics.lateralFluctuationMargin = Distance(lat_fluctuation_margin);

  dynamics.responseTime = Duration(response_time);

  // new parameters after ad-rss v4.0.0
  // leave them as the default one for now
  dynamics.unstructuredSettings.pedestrianTurningRadius = ad::physics::Distance(2.0);
  dynamics.unstructuredSettings.driveAwayMaxAngle = ad::physics::Angle(2.4);
  dynamics.unstructuredSettings.vehicleYawRateChange = ad::physics::AngularAcceleration(0.3);
  dynamics.unstructuredSettings.vehicleMinRadius = ad::physics::Distance(3.5);
  dynamics.unstructuredSettings.vehicleTrajectoryCalculationStep = ad::physics::Duration(0.2);

  // sanity check
  assert(lon_max_brake<=lon_min_brake && lon_min_brake<=lon_min_brake_correct);

  return dynamics;
}

::ad::rss::world::RssDynamics RssInterface::GenerateAgentDynamicsParameters(
    const AgentId &agent_id) {
  if (agents_dynamics_.find(agent_id) != agents_dynamics_.end()) {
    return GenerateVehicleDynamicsParameters(
        agents_dynamics_[agent_id][0], agents_dynamics_[agent_id][1],
        agents_dynamics_[agent_id][2], agents_dynamics_[agent_id][3],
        agents_dynamics_[agent_id][4], agents_dynamics_[agent_id][5],
        agents_dynamics_[agent_id][6], agents_dynamics_[agent_id][7]);
  } else {
    return GenerateVehicleDynamicsParameters(
        default_dynamics_[0], default_dynamics_[1], default_dynamics_[2],
        default_dynamics_[3], default_dynamics_[4], default_dynamics_[5],
        default_dynamics_[6], default_dynamics_[7]);
  }
}

::ad::map::match::Object RssInterface::GenerateMatchObject(
    const models::dynamic::State &agent_state, const Polygon &agent_shape) {
  ::ad::map::match::Object matching_object;
  Point2d agent_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));

  // the calculation is done under ENU coordinate system, it limits the range of
  // input coordinate, work around should be found in the future

  // set matching pose
  matching_object.enuPosition.centerPoint.x =
      ENUCoordinate(static_cast<double>(bg::get<0>(agent_center)));
  matching_object.enuPosition.centerPoint.y =
      ENUCoordinate(static_cast<double>(bg::get<1>(agent_center)));
  matching_object.enuPosition.centerPoint.z = ENUCoordinate(0);
  matching_object.enuPosition.heading = ::ad::map::point::createENUHeading(
      static_cast<double>(agent_state(THETA_POSITION)));

  // set object dimension/boundaries
  matching_object.enuPosition.dimension.length = static_cast<double>(
      Distance(agent_shape.front_dist_ + agent_shape.rear_dist_));
  matching_object.enuPosition.dimension.width = static_cast<double>(
      Distance(agent_shape.left_dist_ + agent_shape.right_dist_));
  matching_object.enuPosition.dimension.height = Distance(2.);
  matching_object.enuPosition.enuReferencePoint =
      ::ad::map::access::getENUReferencePoint();

  // perform map matching
  ::ad::map::match::AdMapMatching map_matching;
  // the last argument is the step size to be used to perform map matching in
  // between the vehicle boundaries, the result of map matching is heavily
  // influencing by this parameter
  matching_object.mapMatchedBoundingBox = map_matching.getMapMatchedBoundingBox(
      matching_object.enuPosition, match_distance);

  return matching_object;
}

::ad::rss::map::RssObjectData RssInterface::GenerateObjectData(
    const ::ad::rss::world::ObjectId& id,
    const ::ad::rss::world::ObjectType& type,
    const ::ad::map::match::Object& matchObject,
    const ::ad::physics::Speed& speed,
    const ::ad::physics::AngularVelocity& yawRate,
    const ::ad::physics::Angle& steeringAngle,
    const ::ad::rss::world::RssDynamics& rssDynamics) {
  ::ad::rss::map::RssObjectData data = {id,      type,          matchObject, speed,
                                   yawRate, steeringAngle, rssDynamics};
  return data;
}

::ad::physics::AngularVelocity RssInterface::CaculateAgentAngularVelocity(
      const models::dynamic::Trajectory& trajectory){
        int n=trajectory.rows();
        if (n<2){
          return 0.f;
        }
        else{
          ::ad::physics::AngularVelocity av =::ad::physics::AngularVelocity(
              trajectory(n - 1, THETA_POSITION) -
              trajectory(0, THETA_POSITION) / trajectory(n - 1, TIME_POSITION) -
              trajectory(0, TIME_POSITION));
          return av;
        }
      }

FullRoute RssInterface::GenerateRoute(
    const Point2d &agent_center,
    const map::LaneCorridorPtr &agent_lane_corridor,
    const ::ad::map::match::Object &match_object) {
  // the center line of lane corridor is used as the target route
  geometry::Line agent_lane_center_line = agent_lane_corridor->GetCenterLine();

  float s_start = GetNearestS(agent_lane_center_line, agent_center);
  float s_end = GetNearestS(
      agent_lane_center_line,
      agent_lane_center_line.obj_.at(agent_lane_center_line.obj_.size() - 1));

  float step = (s_start < s_end) ? discretize_step_ : -discretize_step_;

  std::vector<::ad::map::point::ENUPoint> routing_targets;
  // discretize the line into points and store as routing targets
  while (s_start <= s_end) {
    geometry::Point2d traj_point = GetPointAtS(agent_lane_center_line, s_start);
    routing_targets.push_back(::ad::map::point::createENUPoint(
        bg::get<0>(traj_point), bg::get<1>(traj_point), 0));
    s_start += step;
  }

  std::vector<FullRoute> routes;
  std::vector<double> routes_probability;

  // for each sample point inside the object bounding box generated by map
  // matching, finds a route pass through the routing targets from the sample
  // point
  for (const auto &position :
       match_object.mapMatchedBoundingBox.referencePointPositions[int32_t(
           ::ad::map::match::ObjectReferencePoints::Center)]) {
    auto starting_point = position.lanePoint.paraPoint;
    auto projected_starting_point = starting_point;

    if (!::ad::map::lane::isHeadingInLaneDirection(
            starting_point, match_object.enuPosition.heading)) {
      // project the starting_point to a neighboring lane with the same heading
      // direction as the one of the match object
      ::ad::map::lane::projectPositionToLaneInHeadingDirection(
          starting_point, match_object.enuPosition.heading,
          projected_starting_point);
    }

    auto route_starting_point = ::ad::map::route::planning::createRoutingPoint(
        projected_starting_point, match_object.enuPosition.heading);

    bool valid_route=false;
    if (!routing_targets.empty() &&
        ::ad::map::point::isValid(routing_targets)) {
      FullRoute route = ::ad::map::route::planning::planRoute(
          route_starting_point, routing_targets,
          ::ad::map::route::RouteCreationMode::AllNeighborLanes);

      if (route.roadSegments.size() > 0) {
        valid_route = true;
        routes.push_back(route);
        routes_probability.push_back(position.probability);
      } else {
        LOG(INFO) << "No route to the goal is found at x: "
                  << bg::get<0>(agent_center)
                  << " y: " << bg::get<1>(agent_center) << std::endl;
      }
    }

    if (valid_route==false){
      // predicts all possible routes based on the given distance, it returns
      // all routes having distance less than the given value
      // It should be rarely used
      std::vector<FullRoute> possible_routes =
          ::ad::map::route::planning::predictRoutesOnDistance(
              route_starting_point, Distance(route_predict_range_),
              ::ad::map::route::RouteCreationMode::AllNeighborLanes);
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
    // select the best route based on the probability of the starting point
    // calculated by map matching
    int best_route_idx = std::distance(
        routes_probability.begin(),
        std::max_element(routes_probability.begin(), routes_probability.end()));
    final_route = routes[best_route_idx];
  }
  return final_route;
}

AgentState RssInterface::ConvertAgentState(
    const models::dynamic::State &agent_state,
    const ::ad::rss::world::RssDynamics &agent_dynamics) {
  AgentState rss_state;

  rss_state.timestamp = agent_state(TIME_POSITION);
  rss_state.center = ::ad::map::point::createENUPoint(
      static_cast<double>(agent_state(X_POSITION)),
      static_cast<double>(agent_state(Y_POSITION)), 0.);
  rss_state.heading = ::ad::map::point::createENUHeading(
      static_cast<double>(agent_state(THETA_POSITION)));
  rss_state.speed = Speed(static_cast<double>(agent_state(VEL_POSITION)));
  rss_state.min_stopping_distance =
      CalculateMinStoppingDistance(rss_state.speed, agent_dynamics);

  return rss_state;
}

Distance RssInterface::CalculateMinStoppingDistance(
    const Speed &speed, const ::ad::rss::world::RssDynamics &agent_dynamics) {
  Distance minStoppingDistance;

  bool result =
      ::ad::rss::situation::calculateDistanceOffsetInAcceleratedMovement(
          std::fabs(speed), agent_dynamics.alphaLon.accelMax,
          agent_dynamics.responseTime, minStoppingDistance);

  if (result == false)
    LOG(ERROR)
        << "Failed to calculate maximum possible speed after response time "
        << agent_dynamics.responseTime << std::endl;

  return minStoppingDistance;
}

bool RssInterface::CreateWorldModel(
    const AgentMap& agents, const AgentId& ego_id,
    const AgentState& ego_rss_state,
    const ::ad::map::match::Object& ego_match_object,
    const ::ad::rss::world::RssDynamics& ego_dynamics,
    const ::ad::map::route::FullRoute& ego_route,
    ::ad::rss::world::WorldModel& rss_world_model) {
  std::vector<AgentPtr> relevent_agents;
  double const relevant_distance =
      static_cast<double>(ego_rss_state.min_stopping_distance) *
      checking_relevent_range_;  // increase searching distance for better visualization

  auto ego_av = CaculateAgentAngularVelocity(
      agents.find(ego_id)->second->GetExecutionTrajectory());
  ::ad::rss::map::RssObjectData ego_data = GenerateObjectData(
      ::ad::rss::world::ObjectId(ego_id),
      ::ad::rss::world::ObjectType::EgoVehicle, ego_match_object,
      ego_rss_state.speed, ego_av, ::ad::physics::Angle(ego_rss_state.heading),
      ego_dynamics);

  // determine which agent is close thus relevent for safety checking
  for (const auto &other_agent : agents) {
    if (other_agent.second->GetAgentId() != ego_id) {
      if (geometry::Distance(ego_center,
                             other_agent.second->GetCurrentPosition()) <
          relevant_distance)
        relevent_agents.push_back(other_agent.second);
    }
  }

  ::ad::rss::map::RssSceneCreation scene_creation(ego_rss_state.timestamp,
                                                  ego_dynamics);

  // we don't care about traffic lights right now, but needed by appendScenes.
  ::ad::map::landmark::LandmarkIdSet green_traffic_lights;

  for (const auto &other : relevent_agents) {
    models::dynamic::State other_state;
    try {
      other_state = other->GetCurrentState();
    } catch (const std::exception&) {
      // may fail during initial steps? (during the first 0.5 second)
      return false;
    }

    Polygon other_shape = other->GetShape();
    auto const other_match_object = GenerateMatchObject(
        other_state, other_shape);

    ::ad::rss::world::RssDynamics other_dynamics =
        GenerateAgentDynamicsParameters(other->GetAgentId());
    auto other_av=CaculateAgentAngularVelocity(other->GetExecutionTrajectory());

    ::ad::rss::map::RssObjectData other_data = GenerateObjectData(
        ::ad::rss::world::ObjectId(other->GetAgentId()),
        ::ad::rss::world::ObjectType::OtherVehicle, other_match_object,
        other_state(VEL_POSITION), other_av, ::ad::physics::Angle(other_state(THETA_POSITION)),
        other_dynamics);

    // find all possible scenes between the ego agent and a relevent agent
    scene_creation.appendScenes(ego_data,ego_route,other_data,::ad::rss::map::RssSceneCreation::RestrictSpeedLimitMode::
            ExactSpeedLimit,green_traffic_lights,::ad::rss::map::RssMode::Structured);
  }
  rss_world_model=scene_creation.getWorldModel();

  return withinValidInputRange(rss_world_model);
}

bool RssInterface::RssCheck(
    const ::ad::rss::world::WorldModel& world_model,
    ::ad::rss::state::RssStateSnapshot& rss_state_snapshot) {
  ::ad::rss::core::RssCheck rss_check;
  // describes the relative relation between two objects in possible situations
  ::ad::rss::situation::SituationSnapshot situation_snapshot;

  // contains longitudinal and lateral response of the ego object, a list of id
  // of the dangerous objects
  ::ad::rss::state::ProperResponse proper_response;

  // rss_state_snapshot: individual situation responses calculated from
  // SituationSnapshot
  bool result = rss_check.calculateProperResponse(
      world_model, situation_snapshot, rss_state_snapshot, proper_response);

  if (!result){
    // Due to limitations of Carla map library, RSS check fails when crossing
    // road or road segment and intersection.
    LOG(ERROR) << "Failed to perform RSS check" << std::endl;
  } 
  
  return result;
}

bool RssInterface::ExtractSafetyEvaluation(
    const ::ad::rss::state::RssStateSnapshot &snapshot) {
  bool safety_response = true;
  for (auto const state : snapshot.individualResponses) {
    safety_response = safety_response && !::ad::rss::state::isDangerous(state);
  }
  return safety_response;
}

PairwiseEvaluationReturn RssInterface::ExtractPairwiseSafetyEvaluation(
    const ::ad::rss::state::RssStateSnapshot &snapshot) {
  PairwiseEvaluationReturn pairwise_safety_response;
  for (auto const state : snapshot.individualResponses) {
    pairwise_safety_response[static_cast<AgentId>(state.objectId)] =
        !::ad::rss::state::isDangerous(state);
  }
  return pairwise_safety_response;
}

PairwiseDirectionalEvaluationReturn
RssInterface::ExtractPairwiseDirectionalSafetyEvaluation(
    const ::ad::rss::state::RssStateSnapshot &snapshot) {
  PairwiseDirectionalEvaluationReturn pairwise_safety_response;
  for (auto const state : snapshot.individualResponses) {
    pairwise_safety_response[static_cast<AgentId>(state.objectId)] =
        std::make_pair(::ad::rss::state::isLongitudinalSafe(state),
                       ::ad::rss::state::isLateralSafe(state));
  }
  return pairwise_safety_response;
}

bool RssInterface::ExtractRSSWorld(
    const World& world, const AgentId& agent_id,
    ::ad::rss::world::WorldModel& rss_world_model) {
  AgentPtr agent = world.GetAgent(agent_id);

  models::dynamic::State agent_state;
  agent_state = agent->GetCurrentState();

  Polygon agent_shape = agent->GetShape();
  ::ad::map::match::Object match_object =
      GenerateMatchObject(agent_state, agent_shape);

  Point2d agent_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));
  map::RoadCorridorPtr agent_road_corridor = agent->GetRoadCorridor();
  map::LaneId agent_lane_id = world.GetMap()->FindCurrentLane(agent_center);
  map::LaneCorridorPtr agent_lane_corridor =
      agent_road_corridor->GetLaneCorridor(agent_lane_id);

  ::ad::map::route::FullRoute agent_route =
      GenerateRoute(agent_center, agent_lane_corridor, match_object);

  ::ad::rss::world::RssDynamics agent_dynamics =
      GenerateAgentDynamicsParameters(agent_id);
  AgentState agent_rss_state = ConvertAgentState(agent_state, agent_dynamics);

  AgentMap other_agents = world.GetAgents();
  bool result = CreateWorldModel(other_agents, agent_id, agent_rss_state, match_object,
                   agent_dynamics, agent_route, rss_world_model);

  return result;
}

EvaluationReturn RssInterface::GetSafetyReponse(const World& world,
                                                const AgentId& ego_id) {
  std::optional<bool> response;
  ::ad::rss::world::WorldModel rss_world_model;
  if (ExtractRSSWorld(world, ego_id, rss_world_model)) {
    ::ad::rss::state::RssStateSnapshot snapshot;
    RssCheck(rss_world_model, snapshot);
    response = ExtractSafetyEvaluation(snapshot);
  }
  return response;
}

PairwiseEvaluationReturn RssInterface::GetPairwiseSafetyReponse(
    const World& world, const AgentId& ego_id) {
  ::ad::rss::world::WorldModel rss_world_model;
  PairwiseEvaluationReturn response;
  if (ExtractRSSWorld(world, ego_id, rss_world_model)) {
    ::ad::rss::state::RssStateSnapshot snapshot;
    RssCheck(rss_world_model, snapshot);
    response = ExtractPairwiseSafetyEvaluation(snapshot);
  }
  return response;
}

PairwiseDirectionalEvaluationReturn
RssInterface::GetPairwiseDirectionalSafetyReponse(const World& world,
                                                  const AgentId& ego_id) {
  ::ad::rss::world::WorldModel rss_world_model;
  PairwiseDirectionalEvaluationReturn response;
  if (ExtractRSSWorld(world, ego_id, rss_world_model)) {
    ::ad::rss::state::RssStateSnapshot snapshot;
    RssCheck(rss_world_model, snapshot);
    response = ExtractPairwiseDirectionalSafetyEvaluation(snapshot);
  }
  return response;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark