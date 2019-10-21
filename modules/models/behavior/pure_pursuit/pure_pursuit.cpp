#include "modules/models/behavior/pure_pursuit/pure_pursuit.hpp"


namespace modules {
namespace models {
namespace behavior {

// Trajectory BehaviorPurePursuit::Plan(float delta_time, const ObservedWorld &observed_world) {
//   using namespace dynamic;

//   //! TODO(@fortiss): parameters
//   const int num_traj_time_points = 100;
//   dynamic::Trajectory traj(num_traj_time_points, int(StateDefinition::MIN_STATE_SIZE));
//   auto const sample_time = delta_time / num_traj_time_points;

//   dynamic::State ego_vehicle_state = observed_world.current_ego_state();
//   traj.row(0) = ego_vehicle_state;

//   double start_time = observed_world.get_world_time();
//   geometry::Line line;
//   if (!followed_line_.obj_.empty()) {
//     line = followed_line_;
//   } else {
//     line = observed_world.get_local_map()->get_driving_corridor().get_center();
//   }
//   if (line.obj_.size() > 0) {
//     for (int i = 1; i < num_traj_time_points; ++i) {
//       Input input(2);
//       input << 0.0f, FindSteeringAngle(traj.row(i - 1), line);

//       auto next_state = dynamic::euler_int(single_track_model_, traj.row(i - 1), input, sample_time);

//       traj(i, StateDefinition::TIME_POSITION) = start_time + i*sample_time;
//       traj(i, StateDefinition::X_POSITION) = next_state(StateDefinition::X_POSITION);
//       traj(i, StateDefinition::Y_POSITION) = next_state(StateDefinition::Y_POSITION);
//       traj(i, StateDefinition::THETA_POSITION) = next_state(StateDefinition::THETA_POSITION);
//       traj(i, StateDefinition::VEL_POSITION) = next_state(StateDefinition::VEL_POSITION);
//     }
//   }

//   this->set_last_trajectory(traj);
//   return traj;
// }

double BehaviorPurePursuit::FindSteeringAngle(const dynamic::State &agent_state) const {
  using namespace geometry;
  using namespace dynamic;

  // Point2d nearest_point;
  // float nearest_s;
  // std::tie(nearest_point, nearest_s, std::ignore) = get_nearest_point_and_s(line, agent_position);
  // float s_increment = (line.length() - nearest_s) / 100;

  // float smallest_lookahead_error = std::numeric_limits<float>::max();
  // Point2d lookahead_point;
  // float real_lookahead_distance;
  // for (float s = nearest_s; s <= line.length(); s += s_increment) {
  //   Point2d point_at_s = get_point_at_s(line, s);
  //   float current_distance = distance(point_at_s, nearest_point);
  //   if (fabs(current_distance - lookahead_distance) < smallest_lookahead_error) {
  //     smallest_lookahead_error = fabs(current_distance - lookahead_distance);
  //     lookahead_point = point_at_s;
  //     real_lookahead_distance = current_distance;
  //   }
  // }

  // TODO(@AKreutz): get gain from params
  double gain = 1.0f;

  // TODO(@AKreutz): probably only works for straight lines
  Point2d agent_position(agent_state(StateDefinition::X_POSITION), agent_state(StateDefinition::Y_POSITION));
  double lookahead_distance = gain * agent_state(StateDefinition::VEL_POSITION);
  double nearest_s = get_nearest_s(followed_line_, agent_position);
  Point2d lookahead_point = get_point_at_s(followed_line_, nearest_s + lookahead_distance);

  double lookahead_angle = atan2(bg::get<1>(lookahead_point - agent_position), bg::get<0>(lookahead_point - agent_position)) - agent_state(StateDefinition::THETA_POSITION);

  return atan(2*wheel_base_*sin(lookahead_angle) / distance(agent_position, lookahead_point));
}

} // namespace behavior
} // namespace models
} // namespace modules
