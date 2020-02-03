// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/goal_definition/goal_definition_state_limits_frenet.hpp"
#include "modules/world/objects/agent.hpp"

namespace modules {
namespace world {
namespace goal_definition {


bool GoalDefinitionStateLimitsFrenet::GoalDefinitionStateLimitsFrenet(const modules::geometry::Line& center_line,
                            const std::pair<float,float> max_lateral_distances,
                            const std::pair<float,float> max_orientation_differences,
                            const std::pair<float, float> velocity_range) {
    const modules::geometry::Line line_shifted_left = 
            get_line_shifted_laterally(center_line, - max_lateral_distances.first);
    const modules::geometry::Line line_shifted_right = 
            get_line_shifted_laterally(center_line, max_lateral_distances.second);

    line_shifted_right.reverse();
    line_shifted_left.append_linestring(line_shifted_right)
    const Point2d shape_center_(get_point_at_s(center_line.length()));
    shape_ = Polygon2d(shape_center_, line_shifted_left);
}

bool GoalDefinitionStateLimitsFrenet::AtGoal(
  const modules::world::objects::Agent& agent) {
  const auto agent_state = agent.get_current_state();
  const auto agent_angle = modules::geometry::norm_0_2PI(
    agent_state[modules::models::dynamic::StateDefinition::THETA_POSITION]);
  const modules::geometry::Point2d agent_pos = agent.get_current_position();
  const agent_velocity = agent_state[modules::models::dynamic::StateDefinition::VEL_POSITION];

  if( agent_velocity < velocity_range_.first ||
      agent_velocity > velocity_range_.second) {
        return false;
  }

  if (!modules::geometry::Within(agent_pos, shape_)) {
    return false;
  }
  
  const std::tuple<Point2d, double, uint> nearest_point = get_nearest_point_and_s(center_line_, agent_pos);
  const tangent_angle = get_tangent_angle_at_s(center_line_, std::get<double>(nearest_point));
  const tangent_angle_normalized = modules::geometry::norm_0_2PI(tangent_angle);
  const angle_diff = tangent_angle_normalized - agent_angle;
  
  if (angle_diff <= max_orientation_differences_.first &&
          angle_diff >= - max_orientation_differences_.second ) {
            return true;
  }
  return false;
}


}  // namespace objects
}  // namespace world
}  // namespace modules