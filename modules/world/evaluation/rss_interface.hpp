// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_RSS_INTERFACE_HPP_
#define MODULES_WORLD_EVALUATION_RSS_INTERFACE_HPP_

#include <fstream>
#include <streambuf>
#include <string>

#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/objects/agent.hpp"
#include "modules/world/world.hpp"

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
namespace modules {
namespace world {

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

class RssInterface {
 public:
  RssInterface() {}
  explicit RssInterface(const std::string &opendrive_file_name,
                        const std::vector<float> &default_vehicle_dynamics,
                        const std::unordered_map<AgentId, std::vector<float>>
                            &agent_vehicle_dynamics)
      : default_vehicle_dynamics_(default_vehicle_dynamics),
        agent_vehicle_dynamics_(agent_vehicle_dynamics) {
    spdlog::set_level(spdlog::level::off);
    initializeOpenDriveMap(opendrive_file_name);
  }

  bool GetSafetyReponse(const World &world, const AgentId &ego_id);

  PairwiseEvaluationReturn GetPairwiseSafetyReponse(const World &world,
                                                    const AgentId &ego_id);

  PairwiseDirectionalEvaluationReturn GetPairwiseDirectionalSafetyReponse(
      const World &world, const AgentId &ego_id);

  virtual ~RssInterface() {}

 private:
  bool initializeOpenDriveMap(const std::string &opendrive_file_name);

  ::ad::rss::world::RssDynamics GenerateDefaultVehicleDynamicsParameters();

  ::ad::rss::world::RssDynamics GenerateVehicleDynamicsParameters(
      double lon_max_accel, double lon_max_brake, double lon_min_brake,
      double lon_min_brake_correct, double lat_max_accel, double lat_min_brake,
      double lat_fluctuation_margin, double response_time);

  ::ad::map::match::Object GetMatchObject(
      const models::dynamic::State &agent_state, const Polygon &agent_shape,
      const Distance &match_distance);

  ::ad::map::route::FullRoute GenerateRoute(
      const Point2d &agent_center,
      const map::LaneCorridorPtr &agent_lane_corridor,
      const ::ad::map::match::Object &matched_object);

  AgentState ConvertAgentState(
      const models::dynamic::State &agent_state,
      const ::ad::rss::world::RssDynamics &agent_dynamics);

  ::ad::physics::Distance CalculateMinStoppingDistance(
      const ::ad::physics::Speed &speed,
      const ::ad::rss::world::RssDynamics &agent_dynamics);

  ::ad::rss::world::WorldModel CreateWorldModel(
      const AgentMap &agents, const AgentId &ego_id,
      const AgentState &ego_state,
      const ::ad::map::match::Object &ego_matched_object,
      const ::ad::rss::world::RssDynamics &ego_dynamics,
      const ::ad::map::route::FullRoute &ego_route);

  ::ad::rss::state::RssStateSnapshot RssCheck(
      ::ad::rss::world::WorldModel world_model);

  ::ad::rss::world::WorldModel ExtractRSSWorld(const World &world,
                                               const AgentId &agent_id);

  bool ExtractSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot &snapshot);

  PairwiseEvaluationReturn ExtractPairwiseSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot &snapshot);

  PairwiseDirectionalEvaluationReturn
  ExtractPairwiseDirectionalSafetyEvaluation(
      const ::ad::rss::state::RssStateSnapshot &snapshot);

  std::vector<float> default_vehicle_dynamics_;
  std::unordered_map<AgentId, std::vector<float>> agent_vehicle_dynamics_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_EVALUATION_RSS_INTERFACE_HPP_