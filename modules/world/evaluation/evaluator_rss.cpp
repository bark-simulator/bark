// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/evaluation/evaluator_rss.hpp"

namespace modules {
namespace world {

namespace evaluation {
EvaluationReturn EvaluatorRss::Evaluate(const World& world) {
  AgentPtr ego_agent = world.GetAgent(ego_agent_id_);

  ::ad::map::match::Object ego_matched_object =
      rss_.GetMatchObject(ego_agent, Distance(2.0));

  ::ad::map::route::FullRoute ego_route =
      rss_.GenerateRoute(world, ego_agent_id_, ego_matched_object);

  ::ad::rss::world::RssDynamics ego_dynamics =
      rss_.GenerateDefaultVehicleDynamics();

  Trajectory ego_execut_traj = ego_agent->GetExecutionTrajectory();
  AgentState ego_state =
      rss_.calculateExecutionState(ego_execut_traj, ego_dynamics);

  ::ad::rss::world::WorldModel rss_world_model =
      rss_.createWorldModel(world, ego_agent_id_, ego_state, ego_matched_object,
                            ego_dynamics, ego_route);

  bool is_safe = rss_.RssCheck(rss_world_model);

  return is_safe;
}

}  // namespace evaluation
}  // namespace world
}  // namespace modules