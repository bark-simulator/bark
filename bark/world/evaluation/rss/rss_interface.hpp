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

// An interface that provides a wrapper for the RSS library.
// It provides functionality to convert a BARK into a RSS world and to
// perform a RSS check -- either pairwise or in general for the ego agent.
// Additionally, it also returns the safety response of a specified agent.
//
// RSS: https://github.com/intel/ad-rss-lib
// ad_map_access (dependency of RSS): https://github.com/carla-simulator/map
//
// Carla-RSS integration:
// https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/rss
class RssInterface {
 public:
  RssInterface() {}
  explicit RssInterface(std::string opendrive_file_name,
                        const commons::ParamsPtr& params) {
    acc_lon_max_ = params->GetReal(
      "EvaluatorRss::AccLonMax", "maximum acceleration", 1.7);
    brake_lon_max_ = params->GetReal(
      "EvaluatorRss::BrakeLonMax", "maximum deceleration", -1.7);
    brake_lon_min_ = params->GetReal(
      "EvaluatorRss::BrakeLonMin", "minimum braking deceleration", -1.69);
    brake_lon_min_correct_ = params->GetReal(
      "EvaluatorRss::BrakeLonMinCorrect",
      "minimum deceleration of oncoming vehicle", -1.67);
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
      "Describes the distance for returning all routes.",
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

  /**
   * @brief  Returns a RSS EvaluationReturn for the ObservedWorld
   *         (contains a single ego agent)
   * @note   The function returns false, if the RSS check is found to be unsafe.
   * @param  observed_world: ObservedWorld of an agent's point of view
   * @retval EvaluationReturn
   */
  EvaluationReturn GetSafetyReponse(const ObservedWorld& observed_world);


  /**
   * @brief  Checks the RSS pairwise
   * @note   Returns a PairwiseEvaluationReturn between two specified agents
   * @param  observed_world: ObservedWorld of an agent's point of view
   * @retval PairwiseEvaluationReturn
   */
  PairwiseEvaluationReturn GetPairwiseSafetyReponse(
      const ObservedWorld& observed_world);

  /**
   * @brief  Returns a directional evaluation return.
   * @note   Function is currently not used.
   * @param  observed_world: ObservedWorld of an agent's point of view
   * @retval PairwiseDirectionalEvaluationReturn
   */
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

  // Performs RSS check
  bool RssCheck(const ::ad::rss::world::WorldModel& world_model,
                ::ad::rss::state::RssStateSnapshot& rss_state_snapshot);

  // Generates an RSS world from a BARK world
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

  // minimum allowed braking deceleration in longitudinal direction
  double brake_lon_min_;

  // minimum allowed deceleration in longitudinal direction for a car on its
  // lane with another car approaching on the same lane having an opposite driving
  // direction
  double brake_lon_min_correct_;

  // maximum allowed acceleration in lateral direction
  double acc_lat_brake_max_;

  // minimum allowed braking deceleration in lateral direction
  double acc_lat_brake_min_;

  // fluctuation margin to be considerd for the lateral safe distance
  double fluct_margin_;

  // response time of the ego vehicle
  double response_time_;

  // response time of others
  double response_time_others_;

  // scaling of the relevant range
  double scaling_relevant_range_;

  ::ad::rss::world::RssDynamics rss_dynamics_ego_;
  ::ad::rss::world::RssDynamics rss_dynamics_others_;

  // When a route to the goal cannnot be found, route_predict_range describes
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