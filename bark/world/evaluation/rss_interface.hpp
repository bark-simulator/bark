// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_EVALUATION_RSS_INTERFACE_HPP_
#define BARK_WORLD_EVALUATION_RSS_INTERFACE_HPP_

#include <fstream>
#include <streambuf>
#include <string>
#include <assert.h> 
#include <optional>

#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"
#include "bark/world/map/map_interface.hpp"
#include "bark/world/objects/agent.hpp"
#include "bark/world/world.hpp"

#include <spdlog/spdlog.h>
#include <ad/map/lane/Operation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/match/Object.hpp>
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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

using ::ad::map::point::ENUCoordinate;
using ::ad::physics::Acceleration;
using ::ad::physics::Distance;
using ::ad::physics::Duration;
using ::ad::physics::Speed;

struct AgentState {
  float timestamp;
  Speed speed;
  Distance min_stopping_distance;
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

// Generates a RSS world model from information of agents from BARK, which
// serves as the input of RSS check. Performs RSS check and returns safety
// responses the specifed agent.
//
// Mostly follows the implements of Carla-RSS integration, only supports
// performing RSS on the same road segment and non-intersection area.
//
// Interface for the following libraries:
// RSS: https://github.com/intel/ad-rss-lib
// ad_map_access (dependency of RSS): https://github.com/carla-simulator/map
//
// Carla-RSS integration:
// https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/rss
class RssInterface {
 public:
  RssInterface() {}
  explicit RssInterface(const std::string &opendrive_file_name,
                        const std::vector<float> &default_vehicle_dynamics,
                        const std::unordered_map<AgentId, std::vector<float>>
                            &agent_vehicle_dynamics)
      : default_dynamics_(default_vehicle_dynamics),
        agents_dynamics_(agent_vehicle_dynamics) {
    spdlog::set_level(spdlog::level::off);
    initializeOpenDriveMap(opendrive_file_name);
  }

  // Returns a boolean indicating the safety response of the specified agent.
  // True if for each nearby agents, at least one of the all possible RSS
  // situations is safe, false otherwise.
  EvaluationReturn GetSafetyReponse(const World &world, const AgentId &ego_id);

  // Returns an unorder_map indicating the pairwise safety respone of the
  // specified agent to every other nearby agents. Key is AgentId of an nearby
  // agent, value is true if at least one of the all possible RSS situations
  // between the specified and the nearby agent is safe, false
  // otherwise.
  PairwiseEvaluationReturn GetPairwiseSafetyReponse(const World &world,
                                                    const AgentId &ego_id);

  // Returns an unorder_map indicating the pairwise directional safety respone
  // of the specified agent to every other nearby agents. Key is AgentId of an
  // nearby agent, value is a pair of directional safety response:
  // 
  // 1. longitudinal safety response
  // 2. latitudinal safety response
  // 
  // It is true if at least one of the all possible RSS situations in the
  // direction between the specified and the nearby agent is safe, false
  // otherwise, respectively.
  PairwiseDirectionalEvaluationReturn GetPairwiseDirectionalSafetyReponse(
      const World &world, const AgentId &ego_id);

  virtual ~RssInterface() {}

 private:
  // Load OpenDrive map into RSS, needed for GetMatchObject and GenerateRoute.
  bool initializeOpenDriveMap(const std::string &opendrive_file_name);

  // Generates RSS dynamics for an agent, returns the specified dynamics if the
  // dynamics is given in agent_vehicle_dynamics, else returns the
  // default_vehicle_dynamics
  ::ad::rss::world::RssDynamics GenerateAgentDynamicsParameters(
      const AgentId &agent_id);

  // Creates RSS dynamics object contains the parameters for RSS check.
  ::ad::rss::world::RssDynamics GenerateVehicleDynamicsParameters(
      float lon_max_accel, float lon_max_brake, float lon_min_brake,
      float lon_min_brake_correct, float lat_max_accel, float lat_min_brake,
      float lat_fluctuation_margin, float response_time);

  // Generates a RSS match object from BARK agent state, it contains a list of
  // nearby lanes of the agent state. Used by GenerateRoute and mainly
  // CreateWorldModel.
  //
  // Detailed explanation:
  // https://ad-map-access.readthedocs.io/en/latest/ad_map_access/HLD_MapMatching/
  ::ad::map::match::Object GenerateMatchObject(
      const models::dynamic::State &agent_state, const Polygon &agent_shape,
      const Distance &match_distance);

  ::ad::rss::map::RssObjectData GenerateObjectData(
      const ::ad::rss::world::ObjectId& id,
      const ::ad::rss::world::ObjectType& type,
      const ::ad::map::match::Object& matchObject,
      const ::ad::physics::Speed& speed,
      const ::ad::physics::AngularVelocity& yawRate,
      const ::ad::physics::Angle& steeringAngle,
      const ::ad::rss::world::RssDynamics& rssDynamics);

  ::ad::physics::AngularVelocity CaculateAgentAngularVelocity(
      const models::dynamic::Trajectory& trajectory);

  // Generate a RSS route from the position and lane corridor of the specified
  // BARK agent, the corresponding RSS match object. Used by
  // CreateWorldModel.
  ::ad::map::route::FullRoute GenerateRoute(
      const Point2d &agent_center,
      const map::LaneCorridorPtr &agent_lane_corridor,
      const ::ad::map::match::Object &match_object);

  AgentState ConvertAgentState(
      const models::dynamic::State &agent_state,
      const ::ad::rss::world::RssDynamics &agent_dynamics);

  ::ad::physics::Distance CalculateMinStoppingDistance(
      const ::ad::physics::Speed &speed,
      const ::ad::rss::world::RssDynamics &agent_dynamics);

  // Generates a RSS world model which contains all possible RSS situation
  // between the specified agent and all other nearby agents, from informations
  // generated by other class methods.
  bool CreateWorldModel(const AgentMap& agents, const AgentId& ego_id,
                        const AgentState& ego_state,
                        const ::ad::map::match::Object& ego_match_object,
                        const ::ad::rss::world::RssDynamics& ego_dynamics,
                        const ::ad::map::route::FullRoute& ego_route,
                        ::ad::rss::world::WorldModel& rss_world_model);

  // Placeholder for performing RSS check. Inputs RSS world and returns RSS
  // snapshot which contains the result of RSS check.
  bool RssCheck(const ::ad::rss::world::WorldModel& world_model,
                ::ad::rss::state::RssStateSnapshot& rss_state_snapshot);

  // Extracts RSS world from the information of BARK world, coordinates other functions.
  bool ExtractRSSWorld(const World& world, const AgentId& agent_id,
                       ::ad::rss::world::WorldModel& rss_world_model);

  bool ExtractSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot &snapshot);

  PairwiseEvaluationReturn ExtractPairwiseSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot &snapshot);

  PairwiseDirectionalEvaluationReturn
  ExtractPairwiseDirectionalSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot &snapshot);

  std::vector<float> default_dynamics_;
  std::unordered_map<AgentId, std::vector<float>> agents_dynamics_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_EVALUATION_RSS_INTERFACE_HPP_