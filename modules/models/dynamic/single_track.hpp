// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
#define MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_

#include <algorithm>
#include <memory>
#include "modules/commons/transformation/frenet_state.hpp"
#include "modules/models/dynamic/dynamic_model.hpp"

namespace modules {
namespace models {
namespace dynamic {

class SingleTrackModel : public DynamicModel {
 public:
  explicit SingleTrackModel(const modules::commons::ParamsPtr& params)
      : DynamicModel(params),
        wheel_base_(2.7),
        steering_angle_max_(0.2),
        lat_acceleration_max_(4.0) {
    wheel_base_ = params->GetReal("DynamicModel::wheel_base",
                                  "Wheel base of vehicle [m]", 2.7);
    steering_angle_max_ = params->GetReal("DynamicModel::delta_max",
                                          "Maximum Steering Angle [rad]", 0.2);
    lat_acceleration_max_ =
        params->GetReal("DynamicModel::lat_acc_max",
                        "Maximum lateral acceleration [m/s^2]", 4.0);
  }
  virtual ~SingleTrackModel() {}

  State StateSpaceModel(const State& x, const Input& u) const {
    // TODO(@fortiss): get parameters from Params
    State tmp(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
    tmp << 1,
        x(StateDefinition::VEL_POSITION) *
            cos(x(StateDefinition::THETA_POSITION)),
        x(StateDefinition::VEL_POSITION) *
            sin(x(StateDefinition::THETA_POSITION)),
        x(StateDefinition::VEL_POSITION) * tan(u(1)) / wheel_base_, u(0);
    return tmp;
  }

  std::shared_ptr<DynamicModel> Clone() const {
    std::shared_ptr<SingleTrackModel> model_ptr =
        std::make_shared<SingleTrackModel>(*this);
    return std::dynamic_pointer_cast<DynamicModel>(model_ptr);
  }

  double GetWheelBase() const { return wheel_base_; }
  double GetSteeringAngleMax() const { return steering_angle_max_; }
  double GetLatAccelerationMax() const { return lat_acceleration_max_; }

 private:
  double wheel_base_;
  double steering_angle_max_;
  double lat_acceleration_max_;
};

using SingleTrackModelPtr = std::shared_ptr<SingleTrackModel>;

inline double CalculateSteeringAngle(const SingleTrackModelPtr& model,
                                     const State& state,
                                     const modules::geometry::Line& ref_line,
                                     double gain) {
  // Implemented after G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, and S.
  // Thrun, “Autonomous Automobile Trajectory Tracking for Off-Road Driving:
  // Controller Design, Experimental Validation and Racing,” in 2007 ACC
  //
  // Author: Luis Gressenbuch
  using modules::commons::transformation::FrenetState;
  using StateDefinition::THETA_POSITION;
  using StateDefinition::X_POSITION;
  using StateDefinition::Y_POSITION;

  const double l = model->GetWheelBase();

  // Calculating State of Front Axle
  State state_front(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state_front = state;
  state_front(X_POSITION) = state(X_POSITION) + l * cos(state(THETA_POSITION));
  state_front(Y_POSITION) = state(Y_POSITION) + l * sin(state(THETA_POSITION));

  FrenetState f_state = FrenetState(state_front, ref_line);
  double vel = state(StateDefinition::VEL_POSITION);
  double delta = f_state.angle + atan2(-gain * f_state.lat, vel);

  double delta_max = std::min(
      model->GetSteeringAngleMax(),
      std::abs(std::atan2(
          model->GetLatAccelerationMax() * model->GetWheelBase(), vel * vel)));
  double clamped_delta = std::min(delta, delta_max);
  clamped_delta = std::max(clamped_delta, -delta_max);

  return clamped_delta;
}

}  // namespace dynamic
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
