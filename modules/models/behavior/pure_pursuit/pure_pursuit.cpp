#include "modules/models/behavior/pure_pursuit/pure_pursuit.hpp"


namespace modules {
namespace models {
namespace behavior {

double BehaviorPurePursuit::FindSteeringAngle(const dynamic::State &agent_state) {
  using namespace geometry;
  using dynamic::StateDefinition;

  // TODO(@AKreutz): get gain from params
  double gain = 1.0f;

  // TODO(@AKreutz): probably only works for straight lines
  Point2d agent_position(agent_state(StateDefinition::X_POSITION), agent_state(StateDefinition::Y_POSITION));

  if (!has_reached_line_) {
    if (distance(followed_line_, agent_position) < 0.1f) {
      has_reached_line_ = true;
    }
  }

  double lookahead_distance = gain * agent_state(StateDefinition::VEL_POSITION);
  double nearest_s = get_nearest_s(followed_line_, agent_position);
  Point2d lookahead_point = get_point_at_s(followed_line_, nearest_s + lookahead_distance);

  double lookahead_angle = atan2(bg::get<1>(lookahead_point - agent_position), bg::get<0>(lookahead_point - agent_position)) - agent_state(StateDefinition::THETA_POSITION);

  double delta = atan(2*wheel_base_*sin(lookahead_angle) / distance(agent_position, lookahead_point));

  // TODO(@AKreutz): parameter
  double delta_max = 0.2;
  if (abs(delta) > delta_max) {
    delta = delta > 0 ? delta_max : -delta_max;
  }
  return delta;
}

} // namespace behavior
} // namespace models
} // namespace modules
