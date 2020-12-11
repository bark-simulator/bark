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

    // only adds child if it does not exist; otherwise returns child
    FillRSSDynamics(rss_dynamics_ego_, params->AddChild("EvaluatorRss::Ego"));
    FillRSSDynamics(rss_dynamics_others_, params->AddChild("EvaluatorRss::Others"));

    // general parameters
    scaling_relevant_range_ = params->GetReal(
      "EvaluatorRss::ScalingRelevantRange",
      "Controls the search distance between two agents to perform RSS check",
      1);
    route_predict_range_ = params->GetReal(
      "EvaluatorRss::RoutePredictRange",
      "Describes the distance for returning all routes.",
      50);

    // Sanity check
    // assert(boost::filesystem::exists(opendrive_file_name));
    assert(scaling_relevant_range_ >= 1.);

    // the debugging level in RSS
    spdlog::set_level(spdlog::level::off);
    InitializeOpenDriveMap(opendrive_file_name);
    rss_coordinate_transform_.setENUReferencePoint(
        ::ad::map::access::getENUReferencePoint());
  }

  /**
   * @brief  Fills the RSS libs dynamics
   * @note   
   * @param  rss_dynamics: See RSS documentation 
   * @param  params: child of ParameterServer()
   * @retval None
   */
  void FillRSSDynamics(
    ::ad::rss::world::RssDynamics& rss_dynamics,
    const commons::ParamsPtr& params);
  
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

  // Generates a RSS match object from the BARK agent state. It contains a list of
  // nearby lanes of the agent state. 
  //
  // Detailed explanation:
  // https://ad-map-access.readthedocs.io/en/latest/ad_map_access/HLD_MapMatching/
  ::ad::map::match::Object GenerateMatchObject(
      const models::dynamic::State& agent_state, const Polygon& agent_shape);

  ::ad::physics::AngularVelocity CalculateAngularVelocity(
    const models::behavior::StateActionHistory& history);

  // Generate a RSS route from the current position and the goal of the
  // specified BARK agent and the corresponding RSS match object.
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

	bool longitudinalDistanceOffset(
		const models::dynamic::State& agent_state,
		Distance& distance
	);
  
 private:
  // For a detailed explanation of parameters, please see:
  // https://intel.github.io/ad-rss-lib/ad_rss/Appendix-ParameterDiscussion/#parameter-discussion
  ::ad::rss::world::RssDynamics rss_dynamics_ego_;
  ::ad::rss::world::RssDynamics rss_dynamics_others_;

  double scaling_relevant_range_;
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
