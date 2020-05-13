// Copyright (c) 2020 fortiss GmbH
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "agent_beyond_point_label_evaluator.hpp"

#include "modules/world/observed_world.hpp"
modules::world::evaluation::AgentBeyondPointLabelEvaluator::
    AgentBeyondPointLabelEvaluator(const std::string& string,
                              modules::geometry::Point2d beyond_point)
    : MultiAgentLabelEvaluator(string), beyond_point_(beyond_point) {}
bool modules::world::evaluation::AgentBeyondPointLabelEvaluator::evaluate_agent(
    const modules::world::ObservedWorld& observed_world,
    const modules::world::AgentPtr& other_agent) const {
  const auto agent_pos = other_agent->GetCurrentPosition();
  const auto lc = other_agent->GetRoadCorridor()->GetCurrentLaneCorridor(agent_pos);
  if (lc) {
  FrenetPosition agent_frenet(agent_pos, lc->GetCenterLine());
  FrenetPosition point_frenet(beyond_point_, lc->GetCenterLine());
    return ((agent_frenet.lon - point_frenet.lon) > 0);
  }
  return false;
}
const modules::geometry::Point2d&
modules::world::evaluation::AgentBeyondPointLabelEvaluator::GetBeyondPoint()
    const {
  return beyond_point_;
}
