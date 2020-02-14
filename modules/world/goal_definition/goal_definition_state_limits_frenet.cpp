// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include "modules/world/objects/agent.hpp"

namespace modules {
namespace world {
namespace goal_definition {

namespace bg = boost::geometry;
using modules::geometry::Polygon;
using modules::geometry::Point2d;
using modules::geometry::Line;
using modules::geometry::Pose;
using modules::geometry::Norm0To2PI;
using modules::geometry::GetPointAtS;
using modules::geometry::GetTangentAngleAtS;
using modules::geometry::GetNearestPointAndS;

GoalDefinitionStateLimitsFrenet::GoalDefinitionStateLimitsFrenet(const Line& center_line,
                            const std::pair<float,float> max_lateral_distances,
                            const std::pair<float,float> max_orientation_differences,
                            const std::pair<float, float> velocity_range)  :
                            GoalDefinition(),
                            center_line_(center_line),
                            max_lateral_distances_(max_lateral_distances),
                            max_orientation_differences_(max_orientation_differences),
                            velocity_range_(velocity_range) {
    Line line_shifted_left = 
            GetLineShiftedLaterally(center_line, - max_lateral_distances.first);
    Line line_shifted_right = 
            GetLineShiftedLaterally(center_line, max_lateral_distances.second);

    line_shifted_right.Reverse();
    line_shifted_left.AppendLinestring(line_shifted_right);
    const Point2d shape_center(GetPointAtS(center_line, center_line.Length()));

    shape_ = Polygon(Pose(bg::get<0>(shape_center), bg::get<1>(shape_center), 0), line_shifted_left);
}

bool GoalDefinitionStateLimitsFrenet::AtGoal(
  const modules::world::objects::Agent& agent) {
  const auto agent_state = agent.GetCurrentState();
  const auto agent_angle = Norm0To2PI(
    agent_state[modules::models::dynamic::StateDefinition::THETA_POSITION]);
  const Point2d agent_pos = agent.GetCurrentPosition();
  const auto agent_velocity = agent_state[modules::models::dynamic::StateDefinition::VEL_POSITION];

  if( agent_velocity < velocity_range_.first ||
      agent_velocity > velocity_range_.second) {
        return false;
  }

  if (!modules::geometry::Within(agent_pos, shape_)) {
    return false;
  }

  const std::tuple<Point2d, double, uint> nearest_point = GetNearestPointAndS(center_line_, agent_pos);
  const auto tangent_angle = GetTangentAngleAtS(center_line_, std::get<double>(nearest_point));
  const auto tangent_angle_normalized = Norm0To2PI(tangent_angle);
  const auto angle_diff = tangent_angle_normalized - agent_angle;
  
  if (angle_diff <= max_orientation_differences_.first &&
          angle_diff >= - max_orientation_differences_.second ) {
            return true;
  }
  return false;
}


}  // namespace objects
}  // namespace world
}  // namespace modules