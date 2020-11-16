// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_RSS_INTERFACE_HPP_
#define BARK_WORLD_EVALUATION_RSS_INTERFACE_HPP_

#include <assert.h>
#include <fstream>
#include <optional>
#include <streambuf>
#include <string>

#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/observed_world.hpp"

#include <spdlog/spdlog.h>
#include <ad/map/lane/Operation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/match/Object.hpp>
#include <ad/map/point/CoordinateTransform.hpp>
#include <ad/map/point/ENUOperation.hpp>
#include <ad/map/route/FullRoute.hpp>
#include <ad/map/route/Planning.hpp>
#include <ad/rss/core/RssCheck.hpp>
#include <ad/rss/map/RssSceneCreation.hpp>
#include <ad/rss/situation/Physics.hpp>
#include <ad/rss/situation/SituationSnapshot.hpp>
#include <ad/rss/state/ProperResponse.hpp>
#include <ad/rss/state/RssStateOperation.hpp>
#include <ad/rss/state/RssStateSnapshot.hpp>
#include <ad/rss/world/RssDynamics.hpp>
#include <ad/rss/world/WorldModelValidInputRange.hpp>
#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>

using ::ad::map::point::ENUCoordinate;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;
using ::ad::physics::Speed;

struct AgentState {
  double timestamp;
  Speed speed;
  Distance max_stopping_distance;
  ::ad::map::point::ENUPoint center;
  ::ad::map::point::ENUHeading heading;
};

namespace bg = boost::geometry;
namespace bark {
namespace world {

using geometry::Line;
using geometry::Point2d;
using geometry::Polygon;
using models::dynamic::Trajectory;
using models::dynamic::StateDefinition::THETA_POSITION;
using models::dynamic::StateDefinition::TIME_POSITION;
using models::dynamic::StateDefinition::VEL_POSITION;
using models::dynamic::StateDefinition::X_POSITION;
using models::dynamic::StateDefinition::Y_POSITION;
using objects::AgentPtr;

namespace evaluation {

typedef std::unordered_map<objects::AgentId, bool> PairwiseEvaluationReturn;
typedef std::unordered_map<objects::AgentId, std::pair<bool, bool>>
    PairwiseDirectionalEvaluationReturn;

// This is a library to access RSS library for generating RSS world model from
// information of agents from BARK, which serves as the input of RSS check.
// Performs RSS check and returns safety responses the specifed agent.
// It mostly follows the implements of Carla-RSS integration
//
// RSS: https://github.com/intel/ad-rss-lib
// ad_map_access (dependency of RSS): https://github.com/carla-simulator/map
//
// Carla-RSS integration:
// https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/rss
class RssInterface {
 public:
  RssInterface() {}
  explicit RssInterface(const commons::ParamsPtr& params) {
    std::string opendrive_file_name = params->GetString(
      "EvaluatorRss::MapFilename", "Map path", "");
    acc_lon_max_ = params->GetReal(
      "EvaluatorRss::AccLonMax", "maximum acceleration", 1.7);
    brake_lon_max_ = params->GetReal(
      "EvaluatorRss::BrakeLonMax", "maximum deceleration", -1.7);
    brake_lon_min_ = params->GetReal(
      "EvaluatorRss::BrakeLonMin", "minimum braking deceleration", -1.69);
    brake_lon_min_correct_ = params->GetReal(
      "EvaluatorRss::BrakeLonMinCorrect",
      "minimum deceleration with oncoming vehicle", -1.67);
    acc_lat_brake_max_ = params->GetReal(
      "EvaluatorRss::AccLatBrakeMax", "maximum lateral acceleration", 0.2);
    acc_lat_brake_min_ = params->GetReal(
      "EvaluatorRss::AccLatBrakeMin", "minimum lateral braking", -0.8);
    fluct_margin_ = params->GetReal(
      "EvaluatorRss::FluctMargin", "fluctuation margin", 0.1);
    response_time_ = params->GetReal(
      "EvaluatorRss::TimeResponse", "response time of the ego vehicle", 0.2);
    response_time_others_ = params->GetReal(
      "EvaluatorRss::TimeResponseOthers",
      "response time of other vehicles", 1.0);
    scaling_relevant_range_ = params->GetReal(
      "EvaluatorRss::ScalingRelevantRange",
      "Controls the search distance between two agents to perform RSS check",
      1);
    route_predict_range_ = params->GetReal(
      "EvaluatorRss::RoutePredictRange",
      "Describle the distance for returning all routes "
      "having less than the distance, will be used when "
      "a route to the goal cannnot be found",
      50);
  
    // Sanity checks
    // assert(boost::filesystem::exists(opendrive_file_name));
    assert(scaling_relevant_range_ >= 1.);
    assert(brake_lon_max_ <= brake_lon_min_);
    assert(brake_lon_min_ <= brake_lon_min_correct_);

    rss_dynamics_ego_ = GenerateVehicleDynamicsParameters(response_time_);
    rss_dynamics_others_ = GenerateVehicleDynamicsParameters(response_time_others_);

    // the debugging level in RSS
    spdlog::set_level(spdlog::level::off);
    InitializeOpenDriveMap(opendrive_file_name);
    rss_coordinate_transform_.setENUReferencePoint(
        ::ad::map::access::getENUReferencePoint());
  }

  // Returns a boolean indicating the safety response of the specified agent.
  // True if for each nearby agents, at least one of the two directional RSS
  // situations (longitude and lateral) is safe, false if unsafe, uninitialized
  // (none in python) if rss check can not be performed (only in rare cases).
  // A directional RSS situation considers only the safety in that direction.
  //
  // For example, if the ego agent is following another agent in the same lane
  // at a safe distance, the longitudinal RSS situtation is safe but the
  // lateral one is unsafety.
  EvaluationReturn GetSafetyReponse(const ObservedWorld& observed_world);

  // Returns an unorder_map indicating the pairwise safety respone of the
  // specified agent to every other nearby agents. Key is AgentId of an nearby
  // agent, value ofis true if at least one of the two directional RSS
  // situations between the specified and the nearby agent is safe, false
  // otherwise.
  // Return empty map if no agent is nearby or no Rss check can be performed.
  PairwiseEvaluationReturn GetPairwiseSafetyReponse(
      const ObservedWorld& observed_world);

  // Returns an unorder_map indicating the pairwise directional safety respone
  // of the specified agent to every other nearby agents. Key is AgentId of an
  // nearby agent, value is a pair of directional safety response:
  //
  // 1. longitudinal safety response
  // 2. lateral safety response
  //
  // It is true if at least one of the two directional RSS situations between
  // the specified and the nearby agent is safe, false otherwise, respectively
  // in each direction.
  // Return empty map if no agent is nearby or no Rss check can be performed.
  PairwiseDirectionalEvaluationReturn GetPairwiseDirectionalSafetyReponse(
      const ObservedWorld& observed_world);

  virtual ~RssInterface() {}

  // Load OpenDrive map into RSS, needed for GetMatchObject and GenerateRoute.
  bool InitializeOpenDriveMap(const std::string& opendrive_file_name);

  // Creates RSS dynamics object contains the parameters for RSS check.
  ::ad::rss::world::RssDynamics GenerateVehicleDynamicsParameters(double response_time);

  // Generates a RSS match object from BARK agent state, it contains a list of
  // nearby lanes of the agent state. Used by GenerateRoute and mainly
  // CreateWorldModel.
  //
  // Detailed explanation:
  // https://ad-map-access.readthedocs.io/en/latest/ad_map_access/HLD_MapMatching/
  ::ad::map::match::Object GenerateMatchObject(
      const models::dynamic::State& agent_state, const Polygon& agent_shape);

  ::ad::physics::AngularVelocity CalculateAngularVelocity(
    const models::behavior::StateActionHistory& history);

  // Generate a RSS route from the current position and the goal of the
  // specified BARK agent, the corresponding RSS match object. Used by
  // CreateWorldModel.
  bool GenerateRoute(const Point2d& agent_center, const Point2d& agent_goal,
                     const ::ad::map::match::Object& match_object,
                     ::ad::map::route::FullRoute& route);

  AgentState ConvertAgentState(
      const models::dynamic::State& agent_state,
      const ::ad::rss::world::RssDynamics& agent_dynamics);

  ::ad::physics::Distance CalculateMaxStoppingDistance(
      const ::ad::physics::Speed& speed,
      const ::ad::rss::world::RssDynamics& agent_dynamics);

  // Determine which agent is close thus enough relevent for safety checking
  bool GetRelevantAgents(const AgentMap& agents, const Point2d& ego_center,
                         const AgentId& ego_id,
                         const Distance& ego_max_stopping_distance,
                         std::vector<AgentPtr>& relevant_agents);

  // Generates a RSS world model which contains all possible RSS situation
  // (same/opposite direction, merging/usual scene, intersection scene, etc.)
  // between the specified agent and all other nearby agents, from informations
  // generated by other class methods.
  bool CreateWorldModel(const AgentMap& agents, const AgentId& ego_id,
                        const AgentState& ego_state,
                        const ::ad::map::match::Object& ego_match_object,
                        const ::ad::map::route::FullRoute& ego_route,
                        ::ad::rss::world::WorldModel& rss_world);

  // Placeholder for performing RSS check. Inputs RSS world and returns RSS
  // snapshot which contains the result of RSS check.
  bool RssCheck(const ::ad::rss::world::WorldModel& world_model,
                ::ad::rss::state::RssStateSnapshot& rss_state_snapshot);

  // Generates RSS world from the information of BARK world, coordinates other
  // functions.
  bool GenerateRSSWorld(const ObservedWorld& observed_world,
                        ::ad::rss::world::WorldModel& rss_world);

  bool ExtractSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot& snapshot);

  PairwiseEvaluationReturn ExtractPairwiseSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot& snapshot);

  PairwiseDirectionalEvaluationReturn
  ExtractPairwiseDirectionalSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot& snapshot);

  ::ad::rss::state::ProperResponse GetRSSResponse() const {
    return rss_proper_response_;
  }
  
 private:
  // For a detailed explanation of parameters, please see:
  // https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/#parameter-discussion

  // maximum possible acceleration
  double acc_lon_max_;

  // maximum allowed deceleration in longitudinal direction
  double brake_lon_max_;

  // minimum allowed braking deceleration in longitudinal direction for most
  // scenarios
  double brake_lon_min_;

  // minimum allowed deceleration in longitudinal direction for a car on its
  // lane with another car approaching on the same lane in wrong driving
  // direction
  double brake_lon_min_correct_;

  // maximum allowed acceleration in lateral direction
  double acc_lat_brake_max_;

  // minimum allowed braking deceleration in lateral direction
  double acc_lat_brake_min_;

  // fluctuation margin for that needs to be respected when calculating lateral
  // safe distance
  double fluct_margin_;

  // response time
  double response_time_;

  // response time of other
  double response_time_others_;

  // Scale the searching distance between the evaluating agent and other
  // agents to perform RSS check (used for debugging to have more vehicles being
  // checked)
  double scaling_relevant_range_;

  ::ad::rss::world::RssDynamics rss_dynamics_ego_;
  ::ad::rss::world::RssDynamics rss_dynamics_others_;

  // When a route to the goal cannnot be found, route_predict_range describle
  // the distance for returning all routes having less than the distance
  double route_predict_range_;
  ::ad::map::point::CoordinateTransform rss_coordinate_transform_ =
      ::ad::map::point::CoordinateTransform();
  // Contains longitudinal and lateral response of the ego object, a list of
  // id of the dangerous objects
  ::ad::rss::state::ProperResponse rss_proper_response_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_RSS_INTERFACE_HPP_