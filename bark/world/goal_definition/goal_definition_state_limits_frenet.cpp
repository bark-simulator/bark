// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.
#include "bark/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include <tuple>
#include <utility>
#include "bark/world/objects/agent.hpp"

namespace bark {
namespace world {
namespace goal_definition {

namespace bg = boost::geometry;
using bark::geometry::GetNearestPointAndS;
using bark::geometry::GetPointAtS;
using bark::geometry::GetTangentAngleAtS;
using bark::geometry::Line;
using bark::geometry::Norm0To2PI;
using bark::geometry::Point2d;
using bark::geometry::Polygon;
using bark::geometry::Pose;

GoalDefinitionStateLimitsFrenet::GoalDefinitionStateLimitsFrenet(
    const Line& center_line,
    const std::pair<float, float> max_lateral_distances,
    const std::pair<float, float> max_orientation_differences,
    const std::pair<float, float> velocity_range)
    : GoalDefinition(),
      center_line_(center_line),
      max_lateral_distances_(max_lateral_distances),
      max_orientation_differences_(max_orientation_differences),
      velocity_range_(velocity_range) {
  Line line_shifted_left =
      GetLineShiftedLaterally(center_line, -max_lateral_distances.first);
  Line line_shifted_right =
      GetLineShiftedLaterally(center_line, max_lateral_distances.second);

  line_shifted_right.Reverse();
  line_shifted_left.AppendLinestring(line_shifted_right);
  const Point2d shape_center(GetPointAtS(center_line, center_line.Length()));

  shape_ = Polygon(Pose(bg::get<0>(shape_center), bg::get<1>(shape_center), 0),
                   line_shifted_left);
}

bool GoalDefinitionStateLimitsFrenet::AtGoal(
    const bark::world::objects::Agent& agent) {
  const auto agent_state = agent.GetCurrentState();
  const auto agent_angle = Norm0To2PI(
      agent_state[bark::models::dynamic::StateDefinition::THETA_POSITION]);
  const Point2d agent_pos = agent.GetCurrentPosition();
  const auto agent_velocity =
      agent_state[bark::models::dynamic::StateDefinition::VEL_POSITION];

  if (agent_velocity < velocity_range_.first ||
      agent_velocity > velocity_range_.second) {
    return false;
  }

  if (!bark::geometry::Within(agent_pos, shape_)) {
    return false;
  }

  const std::tuple<Point2d, double, uint> nearest_point =
      GetNearestPointAndS(center_line_, agent_pos);
  const auto tangent_angle =
      GetTangentAngleAtS(center_line_, std::get<double>(nearest_point));
  const auto tangent_angle_normalized = Norm0To2PI(tangent_angle);
  const auto angle_diff = tangent_angle_normalized - agent_angle;

  if (angle_diff <= max_orientation_differences_.first &&
      angle_diff >= -max_orientation_differences_.second) {
    return true;
  }
  return false;
}

}  // namespace goal_definition
}  // namespace world
}  // namespace bark
