// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/rss/rss_interface.hpp"

using ::ad::map::point::ENUCoordinate;
using ::ad::map::route::FullRoute;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;

namespace bark {
namespace world {
namespace evaluation {

bool RssInterface::InitializeOpenDriveMap(
    const std::string& opendrive_file_name) {
  std::ifstream opendrive_file(opendrive_file_name);
  std::string opendrive_file_content =
      std::string{std::istreambuf_iterator<char>(opendrive_file),
                  std::istreambuf_iterator<char>()};
  ::ad::map::access::cleanup();

  // the 2nd argument is the value of narrowing overlapping between two lanes,
  // it is only relevant if the map has intersection
  bool result = ::ad::map::access::initFromOpenDriveContent(
      opendrive_file_content, 0.05,
      ::ad::map::intersection::IntersectionType::Unknown,
      ::ad::map::landmark::TrafficLightType::UNKNOWN);

  if (result == false)
    LOG(ERROR) << "RSS failed to initialize from OpenDrive map : "
               << opendrive_file_content << std::endl;

  return result;
}

::ad::map::match::Object RssInterface::GenerateMatchObject(
    const models::dynamic::State& agent_state, const Polygon& agent_shape) {
  ::ad::map::match::Object matching_object;
  Point2d agent_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));

  // the calculation is done under ENU coordinate system, it limits the range of
  // input coordinate, we should find a work around in the future

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
  matching_object.mapMatchedBoundingBox =
      map_matching.getMapMatchedBoundingBox(matching_object.enuPosition);

  return matching_object;
}

void RssInterface::FillRSSDynamics(
  ::ad::rss::world::RssDynamics& rss_dynamics,
  const commons::ParamsPtr& params) {
  rss_dynamics.alphaLon.accelMax = Acceleration(params->GetReal(
    "AccLonMax", "maximum acceleration", 1.7));
  rss_dynamics.alphaLon.brakeMax = Acceleration(params->GetReal(
    "BrakeLonMax", "maximum deceleration", -1.7));
  rss_dynamics.alphaLon.brakeMin = Acceleration(params->GetReal(
    "BrakeLonMin", "minimum braking deceleration", -1.69));
  rss_dynamics.alphaLon.brakeMinCorrect = Acceleration(params->GetReal(
    "BrakeLonMinCorrect",
    "minimum deceleration of oncoming vehicle", -1.67));
  // lateral
  rss_dynamics.alphaLat.accelMax = Acceleration(params->GetReal(
    "AccLatBrakeMax", "maximum lateral acceleration", 0.2));
  rss_dynamics.alphaLat.brakeMin = Acceleration(params->GetReal(
    "AccLatBrakeMin", "minimum lateral braking", -0.8));
  // other
  rss_dynamics.lateralFluctuationMargin = Distance(params->GetReal(
    "FluctMargin", "fluctuation margin", 0.1));
  rss_dynamics.responseTime = Duration(params->GetReal(
    "TimeResponse", "response time of the ego vehicle", 0.2));

  // new parameters after ad-rss v4.0.0
  rss_dynamics.unstructuredSettings.pedestrianTurningRadius =
    ad::physics::Distance(2.0);
  rss_dynamics.unstructuredSettings.driveAwayMaxAngle = ad::physics::Angle(2.4);
  rss_dynamics.unstructuredSettings.vehicleYawRateChange =
      ad::physics::AngularAcceleration(0.3);
  rss_dynamics.unstructuredSettings.vehicleMinRadius = ad::physics::Distance(3.5);
  rss_dynamics.unstructuredSettings.vehicleTrajectoryCalculationStep =
    ad::physics::Duration(0.2);
  // Sanity check
  // assert(brake_lon_max_ <= brake_lon_min_);
  // assert(brake_lon_min_ <= brake_lon_min_correct_);
}

::ad::physics::AngularVelocity RssInterface::CalculateAngularVelocity(
    const models::behavior::StateActionHistory& history) {
  ::ad::physics::AngularVelocity av;
  if (history.size() < 2) {
    av = ::ad::physics::AngularVelocity(0.0);
  } else {
    models::dynamic::State curr_state = history.at(history.size() - 1).first;
    models::dynamic::State prev_state = history.at(history.size() - 2).first;
    double diff_theta = curr_state(THETA_POSITION) - prev_state(THETA_POSITION);
    double diff_time = curr_state(TIME_POSITION) - prev_state(TIME_POSITION);
    av = ::ad::physics::AngularVelocity(diff_theta / diff_time);
  }
  return av;
}

bool RssInterface::GenerateRoute(const Point2d& agent_center,
                                 const Point2d& agent_goal,
                                 const ::ad::map::match::Object& match_object,
                                 FullRoute& route) {
  bool is_valid_route_found = false;
  std::vector<FullRoute> routes;
  std::vector<double> routes_probability;

  // For each sample point inside the object bounding box generated by map
  // matching, finds a route pass through the routing targets from the sample
  // point
  for (const auto& position :
       match_object.mapMatchedBoundingBox.referencePointPositions[int32_t(
           ::ad::map::match::ObjectReferencePoints::Center)]) {
    auto agent_ENU_goal = ::ad::map::point::createENUPoint(
        static_cast<double>(bg::get<0>(agent_goal)),
        static_cast<double>(bg::get<1>(agent_goal)), 0.);
    auto agent_geo_goal = rss_coordinate_transform_.ENU2Geo(agent_ENU_goal);

    auto agent_parapoint = position.lanePoint.paraPoint;
    auto projected_starting_point = agent_parapoint;
    if (!::ad::map::lane::isHeadingInLaneDirection(
            agent_parapoint, match_object.enuPosition.heading)) {
      // project the agent_parapoint to a neighboring lane with the same heading
      // direction as the one of the match object
      ::ad::map::lane::projectPositionToLaneInHeadingDirection(
          agent_parapoint, match_object.enuPosition.heading,
          projected_starting_point);
    }

    auto route_starting_point = ::ad::map::route::planning::createRoutingPoint(
        projected_starting_point, match_object.enuPosition.heading);

    FullRoute route = ::ad::map::route::planning::planRoute(
        route_starting_point, agent_geo_goal,
        ::ad::map::route::RouteCreationMode::AllRoutableLanes);

    if (route.roadSegments.size() > 0) {
      is_valid_route_found = true;
      routes.push_back(route);
      routes_probability.push_back(position.probability);
    } else {
      // If no route is found, the RSS check may not be accurate anymore
      LOG(WARNING) << "No 00route to the goal is found:\nCurrent location x : "
                   << bg::get<0>(agent_center)
                   << " y: " << bg::get<1>(agent_center)
                   << "\nGoal: x: " << bg::get<0>(agent_goal)
                   << " y: " << bg::get<1>(agent_goal) << std::endl;
    }

    if (is_valid_route_found == false) {
      // Guess all possible routes based on a given distance, it returns
      // all routes having distance less than the given value
      // (This case should't happen)
      std::vector<FullRoute> possible_routes =
          ::ad::map::route::planning::predictRoutesOnDistance(
              route_starting_point, Distance(route_predict_range_),
              ::ad::map::route::RouteCreationMode::AllNeighborLanes);
      for (const auto& possible_route : possible_routes) {
        routes.push_back(possible_route);
        routes_probability.push_back(0);
      }
    }
  }

  if (routes.empty()) {
    LOG(WARNING) << "Could not find any route from the agent current position "
                    "to the goal"
                 << std::endl;
  } else {
    // Select the best route based on the probability of the starting point
    // calculated by map matching
    int best_route_idx = std::distance(
        routes_probability.begin(),
        std::max_element(routes_probability.begin(), routes_probability.end()));
    route = routes[best_route_idx];
  }
  return is_valid_route_found;
}

AgentState RssInterface::ConvertAgentState(
    const models::dynamic::State& agent_state,
    const ::ad::rss::world::RssDynamics& agent_dynamics) {
  AgentState rss_state;

  rss_state.timestamp = agent_state(TIME_POSITION);
  rss_state.center = ::ad::map::point::createENUPoint(
      static_cast<double>(agent_state(X_POSITION)),
      static_cast<double>(agent_state(Y_POSITION)), 0.);
  rss_state.heading = ::ad::map::point::createENUHeading(
      static_cast<double>(agent_state(THETA_POSITION)));
  rss_state.speed = Speed(static_cast<double>(agent_state(VEL_POSITION)));

  rss_state.max_stopping_distance =
      CalculateMaxStoppingDistance(rss_state.speed, agent_dynamics);

  return rss_state;
}

Distance RssInterface::CalculateMaxStoppingDistance(
    const Speed& speed, const ::ad::rss::world::RssDynamics& agent_dynamics) {
  // Estimate the upper bound of the unsafe distance according to the Rss paper.
  // The calculated value is always higher than the correct upper bound.

  Distance braking_distance_after_responsing = 50,
           traveled_distance_when_responsing = 50;
  Speed speed_when_responsing;

  Acceleration max_accel = std::max(agent_dynamics.alphaLon.accelMax,
                                    agent_dynamics.alphaLat.accelMax);
  Acceleration min_brake = std::min(agent_dynamics.alphaLon.brakeMin,
                                    agent_dynamics.alphaLon.brakeMax);

  // Assume every agent has no maximum speed limit
  bool result = ::ad::rss::situation::calculateAcceleratedLimitedMovement(
      std::fabs(speed), Speed::getMax(), max_accel, agent_dynamics.responseTime,
      speed_when_responsing, traveled_distance_when_responsing);

  result = result && ::ad::rss::situation::calculateStoppingDistance(
                         std::fabs(speed_when_responsing), min_brake,
                         braking_distance_after_responsing);

  if (result == false)
    LOG(ERROR) << "Failed to calculate maximum stopping distance" << std::endl;

  return traveled_distance_when_responsing + braking_distance_after_responsing;
}

bool RssInterface::GetRelevantAgents(const AgentMap& agents,
                                     const Point2d& ego_center,
                                     const AgentId& ego_id,
                                     const Distance& ego_max_stopping_distance,
                                     std::vector<AgentPtr>& relevant_agents) {
  for (const auto& other_agent : agents) {
    AgentId other_agent_id = other_agent.second->GetAgentId();
    if (other_agent_id != ego_id) {
      double other_agent_speed =
          other_agent.second->GetCurrentState()(VEL_POSITION);
      double relevant_distance =
          (static_cast<double>(ego_max_stopping_distance) +
           CalculateMaxStoppingDistance(other_agent_speed, rss_dynamics_others_)) *
          scaling_relevant_range_;

      if (geometry::Distance(ego_center,
                             other_agent.second->GetCurrentPosition()) <
          relevant_distance)
        relevant_agents.push_back(other_agent.second);
    }
  }
  return true;
}

bool RssInterface::CreateWorldModel(
    const AgentMap& agents, const AgentId& ego_id,
    const AgentState& ego_rss_state,
    const ::ad::map::match::Object& ego_match_object,
    const ::ad::map::route::FullRoute& ego_route,
    ::ad::rss::world::WorldModel& rss_world) {
  geometry::Point2d ego_center(ego_rss_state.center.x, ego_rss_state.center.y);
  auto ego_av = CalculateAngularVelocity(
      agents.find(ego_id)->second->GetStateInputHistory());
  ::ad::rss::map::RssObjectData ego_data = {
      ::ad::rss::world::ObjectId(ego_id),
      ::ad::rss::world::ObjectType::EgoVehicle,
      ego_match_object,
      ego_rss_state.speed,
      ego_av,
      ::ad::physics::Angle(ego_rss_state.heading),
      rss_dynamics_ego_};

  std::vector<AgentPtr> relevant_agents;
  GetRelevantAgents(agents, ego_center, ego_id,
                    ego_rss_state.max_stopping_distance, relevant_agents);

  // +1 is a work around because RSS defines the world is only valid after
  // world_time >=1
  ::ad::rss::map::RssSceneCreation scene_creation(ego_rss_state.timestamp + 1,
                                                  rss_dynamics_ego_);

  // It is not relevant, but needed by appendScenes.
  ::ad::map::landmark::LandmarkIdSet green_traffic_lights;

  for (const auto& other : relevant_agents) {
    models::dynamic::State other_state = other->GetCurrentState();

    Polygon other_shape = other->GetShape();
    auto const other_match_object =
        GenerateMatchObject(other_state, other_shape);

    auto other_av = CalculateAngularVelocity(other->GetStateInputHistory());

    ::ad::rss::map::RssObjectData other_data = {
        ::ad::rss::world::ObjectId(other->GetAgentId()),
        ::ad::rss::world::ObjectType::OtherVehicle,
        other_match_object,
        other_state(VEL_POSITION),
        other_av,
        ::ad::physics::Angle(other_state(THETA_POSITION)),
        rss_dynamics_others_};

    // Find all possible scenes between the ego agent and a relevant agent
    scene_creation.appendScenes(ego_data, ego_route, other_data,
                                ::ad::rss::map::RssSceneCreation::
                                    RestrictSpeedLimitMode::ExactSpeedLimit,
                                green_traffic_lights,
                                ::ad::rss::map::RssMode::Structured);
  }
  rss_world = scene_creation.getWorldModel();

  // It is valid only after world timestep 1
  bool result = withinValidInputRange(rss_world);
  return result;
}

bool RssInterface::RssCheck(
    const ::ad::rss::world::WorldModel& world_model,
    ::ad::rss::state::RssStateSnapshot& rss_state_snapshot) {
  ::ad::rss::core::RssCheck rss_check;
  // Describes the relative relation between two objects in possible
  // situations
  ::ad::rss::situation::SituationSnapshot situation_snapshot;

  // rss_state_snapshot: individual situation responses calculated from
  // SituationSnapshot
  bool result = rss_check.calculateProperResponse(
      world_model, situation_snapshot, rss_state_snapshot,
      rss_proper_response_);

  if (!result) {
    LOG(ERROR) << "Failed to perform RSS check" << std::endl;
  }

  return result;
}

bool RssInterface::ExtractSafetyEvaluation(
    const ::ad::rss::state::RssStateSnapshot& snapshot) {
  bool is_safe = true;
  for (auto const state : snapshot.individualResponses) {
    is_safe = is_safe && !::ad::rss::state::isDangerous(state);
  }
  return is_safe;
}

PairwiseEvaluationReturn RssInterface::ExtractPairwiseSafetyEvaluation(
    const ::ad::rss::state::RssStateSnapshot& snapshot) {
  PairwiseEvaluationReturn is_pairwise_safe;
  for (auto const state : snapshot.individualResponses) {
    is_pairwise_safe[static_cast<AgentId>(state.objectId)] =
        !::ad::rss::state::isDangerous(state);
  }
  return is_pairwise_safe;
}

PairwiseDirectionalEvaluationReturn
RssInterface::ExtractPairwiseDirectionalSafetyEvaluation(


    const ::ad::rss::state::RssStateSnapshot& snapshot) {
      
  PairwiseDirectionalEvaluationReturn is_pairwise_directionally_safe;
  for (auto const state : snapshot.individualResponses) {
    is_pairwise_directionally_safe[static_cast<AgentId>(state.objectId)] =
        std::make_pair(::ad::rss::state::isLongitudinalSafe(state),
                       ::ad::rss::state::isLateralSafe(state));

        // use std::tuple instead of make_pair and return numerical values for 
        // the violations
  }
  return is_pairwise_directionally_safe;
}

bool RssInterface::GenerateRSSWorld(const ObservedWorld& observed_world,
                                    ::ad::rss::world::WorldModel& rss_world) {
  AgentPtr agent = observed_world.GetEgoAgent();
  AgentId agent_id = observed_world.GetEgoAgentId();

  Point2d agent_goal;
  bg::centroid(agent->GetGoalDefinition()->GetShape().obj_, agent_goal);
  models::dynamic::State agent_state;
  agent_state = agent->GetCurrentState();
  Polygon agent_shape = agent->GetShape();
  Point2d agent_center =
      Point2d(agent_state(X_POSITION), agent_state(Y_POSITION));

  ::ad::map::match::Object agent_match_object =
      GenerateMatchObject(agent_state, agent_shape);
  ::ad::map::route::FullRoute agent_rss_route;
  GenerateRoute(agent_center, agent_goal, agent_match_object, agent_rss_route);
  AgentState agent_rss_state = ConvertAgentState(agent_state, rss_dynamics_ego_);

  AgentMap other_agents = observed_world.GetAgents();  // GetOtherAgents();
  bool result = CreateWorldModel(other_agents, agent_id, agent_rss_state,
                                 agent_match_object,
                                 agent_rss_route, rss_world);

  return result;
}

EvaluationReturn RssInterface::GetSafetyReponse(
    const ObservedWorld& observed_world) {
  std::optional<bool> response;
  ::ad::rss::world::WorldModel rss_world;
  if (GenerateRSSWorld(observed_world, rss_world)) {
    ::ad::rss::state::RssStateSnapshot snapshot;
    RssCheck(rss_world, snapshot);
    response = ExtractSafetyEvaluation(snapshot);
    // Works until here, but response always returns true
    
    
    
  }
  return response;
}

PairwiseEvaluationReturn RssInterface::GetPairwiseSafetyReponse(
    const ObservedWorld& observed_world) {
  ::ad::rss::world::WorldModel rss_world;
  PairwiseEvaluationReturn response;
  if (GenerateRSSWorld(observed_world, rss_world)) {
    ::ad::rss::state::RssStateSnapshot snapshot;
    RssCheck(rss_world, snapshot);
    response = ExtractPairwiseSafetyEvaluation(snapshot);
  }
  return response;
}

PairwiseDirectionalEvaluationReturn
RssInterface::GetPairwiseDirectionalSafetyReponse(
    const ObservedWorld& observed_world) {
  ::ad::rss::world::WorldModel rss_world;
  PairwiseDirectionalEvaluationReturn response;
  if (GenerateRSSWorld(observed_world, rss_world)) {
    ::ad::rss::state::RssStateSnapshot snapshot;
    RssCheck(rss_world, snapshot);
    response = ExtractPairwiseDirectionalSafetyEvaluation(snapshot);
    
    Distance lat_distance = 0;
    Distance long_distance =0;
    AgentPtr agent = observed_world.GetEgoAgent();
    models::dynamic::State agent_state;
    agent_state = agent->GetCurrentState();

    bool latResp =  lateralDistanceOffset(agent_state, lat_distance); 
    bool longResp = longitudinalDistanceOffset(agent_state, long_distance); 
    std::cout << lat_distance << " Lat" << std::endl;
    std::cout << long_distance << " Long" << std::endl;
  }
  // TODO: change it to a tuple
  return response;
}

bool
RssInterface::lateralDistanceOffset(
		const models::dynamic::State& agent_state,
		Distance& distance) {
  const AgentState rss_state = ConvertAgentState(agent_state, rss_dynamics_ego_);
  bool resp = ::ad::rss::situation::calculateLateralDistanceOffsetAfterStatedBrakingPattern(
  rss_state.speed,
  rss_dynamics_ego_.responseTime,
  rss_dynamics_ego_.alphaLon.accelMax,
  rss_dynamics_ego_.alphaLon.brakeMax,
  distance);
        //Speed const &currentSpeed,
        //Duration const &responseTime,
         //Acceleration const &acceleration,
         ///Acceleration const &deceleration,
         //Distance &distanceOffset
  
  return resp;
}

bool
RssInterface::longitudinalDistanceOffset(
		const models::dynamic::State& agent_state,
		Distance& distance) {
  const AgentState rss_state =ConvertAgentState(agent_state, rss_dynamics_ego_);

	return ::ad::rss::situation::calculateLongitudinalDistanceOffsetAfterStatedBrakingPattern(
		rss_state.speed,
		Speed::getMax(),
	  rss_dynamics_ego_.responseTime,
		rss_dynamics_ego_.alphaLon.accelMax,
		rss_dynamics_ego_.alphaLon.brakeMax,
      distance
	);
}






}  // namespace evaluation
}  // namespace world
}  // namespace bark