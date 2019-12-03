#ifndef MODULES_MODELS_BEHAVIOR_PURE_PURSUIT_PURE_PURSUIT_HPP_
#define MODULES_MODELS_BEHAVIOR_PURE_PURSUIT_PURE_PURSUIT_HPP_

#include "modules/geometry/line.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"


namespace modules {
namespace models {
namespace behavior {

// From technical report "Implementation of the Pure Pursuit Path Tracking Algorithm"
class BehaviorPurePursuit {
  public:
    explicit BehaviorPurePursuit(commons::Params *params) {
      wheel_base_ = params->get_real("DynamicModel::wheel_base", "Wheel base of vehicle.", 2.7);
    }

    double FindSteeringAngle(const dynamic::State &agent_state);

    void set_followed_line(const geometry::Line followed_line) {
      followed_line_ = followed_line;
      has_reached_line_ = false;
    }
    bool is_following_line() const { return followed_line_.size() != 0; }
    bool has_reached_line() const { return has_reached_line_; }

  private:
    geometry::Line followed_line_;

    bool has_reached_line_;

    float wheel_base_;
};

} // namespace behavior
} // namespace models
} // namespace modules

#endif // MODULES_MODELS_BEHAVIOR_PURE_PURSUIT_PURE_PURSUIT_HPP_
