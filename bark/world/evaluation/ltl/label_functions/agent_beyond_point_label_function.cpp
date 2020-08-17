// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "agent_beyond_point_label_function.hpp"

#include "bark/world/observed_world.hpp"
bark::world::evaluation::AgentBeyondPointLabelFunction::
    AgentBeyondPointLabelFunction(const std::string& string,
                                  bark::geometry::Point2d beyond_point)
    : MultiAgentLabelFunction(string), beyond_point_(beyond_point) {}
bool bark::world::evaluation::AgentBeyondPointLabelFunction::EvaluateAgent(
    const bark::world::ObservedWorld& observed_world,
    const bark::world::AgentPtr& other_agent) const {
  const auto agent_pos = other_agent->GetCurrentPosition();
  const auto lc =
      other_agent->GetRoadCorridor()->GetCurrentLaneCorridor(agent_pos);
  if (lc) {
    FrenetPosition agent_frenet(agent_pos, lc->GetCenterLine());
    FrenetPosition point_frenet(beyond_point_, lc->GetCenterLine());
    return ((agent_frenet.lon - point_frenet.lon) > 0);
  }
  return false;
}
const bark::geometry::Point2d&
bark::world::evaluation::AgentBeyondPointLabelFunction::GetBeyondPoint() const {
  return beyond_point_;
}
