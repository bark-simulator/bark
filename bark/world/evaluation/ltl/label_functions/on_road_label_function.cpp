// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/evaluation/ltl/label_functions/on_road_label_function.hpp"

#include "bark/world/observed_world.hpp"

namespace bark {
namespace world {
namespace evaluation {

using bark::geometry::Polygon;

OnRoadLabelFunction::OnRoadLabelFunction(const std::string& label_str)
    : MultiAgentLabelFunction(label_str) {}

bool OnRoadLabelFunction::EvaluateAgent(
    const world::ObservedWorld& observed_world,
    const AgentPtr& other_agent) const {
  const auto& pos = other_agent->GetCurrentPosition();

  auto lane_corr = other_agent->GetRoadCorridor()->GetNearestLaneCorridor(pos);
  bool is_on_road = false;
  if (lane_corr) {
    const Polygon& corridor_polygon = lane_corr->GetMergedPolygon();
    if (bark::geometry::Collide(corridor_polygon, pos)) {
      is_on_road = true;
    }
  }
  return is_on_road;
}

}  // namespace evaluation
}  // namespace world
}  // namespace bark