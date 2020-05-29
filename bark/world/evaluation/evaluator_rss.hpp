// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// #ifndef MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
// #define MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_

#include <fstream>
#include <limits>
#include <memory>
#include <streambuf>
#include <string>

#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"
#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/map/map_interface.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/world/world.hpp"

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <ad/map/access/Logging.hpp>
#include <ad/map/access/Operation.hpp>
#include <ad/map/intersection/Intersection.hpp>
#include <ad/map/landmark/LandmarkIdSet.hpp>
#include <ad/map/lane/Operation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/match/MapMatchedOperation.hpp>
#include <ad/map/match/Object.hpp>
#include <ad/map/point/ENUOperation.hpp>
#include <ad/map/point/HeadingOperation.hpp>
#include <ad/map/route/FullRoute.hpp>
#include <ad/map/route/LaneIntervalOperation.hpp>
#include <ad/map/route/Operation.hpp>
#include <ad/map/route/Planning.hpp>
#include <ad/rss/core/RssCheck.hpp>
#include <ad/rss/map/Logging.hpp>
#include <ad/rss/map/RssObjectConversion.hpp>
#include <ad/rss/map/RssSceneCreation.hpp>
#include <ad/rss/map/RssSceneCreator.hpp>
#include <ad/rss/situation/SituationSnapshot.hpp>
#include <ad/rss/state/ProperResponse.hpp>
#include <ad/rss/state/RssStateOperation.hpp>
#include <ad/rss/state/RssStateSnapshot.hpp>
#include <ad/rss/world/Object.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

struct EgoDynamicsOnRoute {
  /// @brief constructor
  EgoDynamicsOnRoute();

  /// @brief the carla timestamp of the last calculation
  float timestamp;
  /// @brief the time since epoch in ms at start of the checkObjects call
  double time_since_epoch_check_start_ms;
  /// @brief the time since epoch in ms at the end of the checkObjects call
  double time_since_epoch_check_end_ms;
  /// @brief the ego speed
  ::ad::physics::Speed ego_speed;
  /// @brief the current minimum stopping distance
  ::ad::physics::Distance min_stopping_distance;
  /// @brief the considered enu position of the ego vehicle
  ::ad::map::point::ENUPoint ego_center;
  /// @brief the considered heading of the ego vehicle
  ::ad::map::point::ENUHeading ego_heading;
  /// @brief check if the ego center is within route
  bool ego_center_within_route;
  /// @brief flag indicating if the current state is already crossing one of the
  /// borders
  /// this is only evaluated if the border checks are active!
  /// It is a hint to oversteer a bit on countersteering
  bool crossing_border;
  /// @brief the considered heading of the route
  ::ad::map::point::ENUHeading route_heading;
  /// @brief the considered nominal center of the current route
  ::ad::map::point::ENUPoint route_nominal_center;
  /// @brief the considered heading diff towards the route
  ::ad::map::point::ENUHeading heading_diff;
  /// @brief the ego speed component lat in respect to a route
  ::ad::physics::Speed route_speed_lat;
  /// @brief the ego speed component lon in respect to a route
  ::ad::physics::Speed route_speed_lon;
  /// @brief the ego acceleration component lat in respect to a route
  ::ad::physics::Acceleration route_accel_lat;
  /// @brief the ego acceleration component lon in respect to a route
  ::ad::physics::Acceleration route_accel_lon;
  /// @brief the ego acceleration component lat in respect to a route
  /// smoothened by an average filter
  ::ad::physics::Acceleration avg_route_accel_lat;
  /// @brief the ego acceleration component lon in respect to a route
  /// smoothened by an average filter
  ::ad::physics::Acceleration avg_route_accel_lon;
};

struct CarlaRssState {
  /// @brief the actual RSS checker object
  ::ad::rss::core::RssCheck rss_check;

  /// @brief the ego map matched information
  ::ad::map::match::Object ego_match_object;

  /// @brief the ego route
  ::ad::map::route::FullRoute ego_route;

  /// @brief the ego dynamics on the route
  EgoDynamicsOnRoute ego_dynamics_on_route;

  /// @brief check input: the RSS world model
  ::ad::rss::world::WorldModel world_model;

  /// @brief check result: the situation snapshot
  ::ad::rss::situation::SituationSnapshot situation_snapshot;
  /// @brief check result: the rss state snapshot
  ::ad::rss::state::RssStateSnapshot rss_state_snapshot;
  /// @brief check result: the proper response
  ::ad::rss::state::ProperResponse proper_response;
  /// @brief check result: the acceleration restrictions
  ::ad::rss::world::AccelerationRestriction acceleration_restriction;
  /// @brief flag indicating if the current state is overall dangerous
  bool dangerous_state;
  /// @brief flag indicating if the current state is dangerous because of a
  /// vehicle
  bool dangerous_vehicle;
  /// @brief flag indicating if the current state is dangerous because of an
  /// opposite vehicle
  bool dangerous_opposite_state;
};

namespace modules {
namespace world {
namespace evaluation {

class EvaluatorRss : public BaseEvaluator {
 public:
  EvaluatorRss() : agent_id_(std::numeric_limits<AgentId>::max()) {}
  explicit EvaluatorRss(const AgentId& agent_id,
                        const std::string& opendrive_file_path)
      : agent_id_(agent_id) {
    std::ifstream file(opendrive_file_path);
        std::string opendrive_file_content = std::string{
        std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};


    ::ad::map::access::cleanup();
    std::cout<< "Initialize opendrive map: "<< ::ad::map::access::initFromOpenDriveContent(
        opendrive_file_content, 0.2,
        ::ad::map::intersection::IntersectionType::TrafficLight,
        ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN) <<std::endl;

    // TODO: move to a function
    default_dynamics_.alphaLon.accelMax = ::ad::physics::Acceleration(3.5);
    default_dynamics_.alphaLon.brakeMax = ::ad::physics::Acceleration(-8.);
    default_dynamics_.alphaLon.brakeMin = ::ad::physics::Acceleration(-4.);
    default_dynamics_.alphaLon.brakeMinCorrect =
        ::ad::physics::Acceleration(-3);
    default_dynamics_.alphaLat.accelMax = ::ad::physics::Acceleration(0.2);
    default_dynamics_.alphaLat.brakeMin = ::ad::physics::Acceleration(-0.8);
    default_dynamics_.lateralFluctuationMargin = ::ad::physics::Distance(0.1);
    default_dynamics_.responseTime = ::ad::physics::Duration(1.0);


  }
  virtual ~EvaluatorRss() {}
  virtual EvaluationReturn Evaluate(const world::World& world);

 private:
  AgentId agent_id_;
  EgoDynamicsOnRoute ego_dynamics_on_route;
  CarlaRssState _carla_rss_state;

  ::ad::rss::world::RssDynamics default_dynamics_;
};

}  // namespace evaluation
}  // namespace world
}  // namespace modules

// #endif  // MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
