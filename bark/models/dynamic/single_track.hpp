// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
#define BARK_MODELS_DYNAMIC_SINGLE_TRACK_HPP_

#include <algorithm>
#include <memory>
#include "bark/commons/transformation/frenet_state.hpp"
#include "bark/models/dynamic/dynamic_model.hpp"

namespace bark {
namespace models {
namespace dynamic {

struct AccelerationLimits {
  double lat_acc_left_max;
  double lat_acc_right_max;
  double lon_acc_max;
  double lon_acc_min;
};

inline std::ostream& operator<<(std::ostream& os,
                                const AccelerationLimits& al) {
  os << "AccelerationLimits = ("
     << " lat_acc_left_max: " << al.lat_acc_left_max << ", "
     << " lat_acc_right_max: " << al.lat_acc_right_max << ", "
     << " lon_acc_max: " << al.lon_acc_max << ", "
     << " lon_acc_min: " << al.lon_acc_min << ")";
  return os;
}

inline AccelerationLimits AccelerationLimitsFromParamServer(
    const bark::commons::ParamsPtr& params) {
  AccelerationLimits al = AccelerationLimits();
  al.lat_acc_left_max =
      params->GetReal("DynamicModel::LatAccLeftMax",
                      "Maximum lateral left acceleration [m/s^2]", 4.0);
  al.lat_acc_right_max =
      params->GetReal("DynamicModel::LatAccRightMax",
                      "Maximum lateral right acceleration [m/s^2]", 4.0);
  al.lon_acc_max = params->GetReal("DynamicModel::LonAccelerationMax",
                                   "Maximum longitudinal acceleration", 4.0);
  al.lon_acc_min = params->GetReal("DynamicModel::LonAccelerationMin",
                                   "Minimum longitudinal acceleration", -8.0);
  return al;
}

class SingleTrackModel : public DynamicModel {
 public:
  explicit SingleTrackModel(const bark::commons::ParamsPtr& params)
      : DynamicModel(params),
        wheel_base_(params->GetReal("DynamicModel::WheelBase",
                                    "Wheel base of vehicle [m]", 2.7)),
        steering_angle_max_(params->GetReal(
            "DynamicModel::DeltaMax", "Maximum Steering Angle [rad]", 0.2)) {
    acceleration_limits_ = AccelerationLimitsFromParamServer(params);
  }
  virtual ~SingleTrackModel() {}

  State StateSpaceModel(const State& x, const Input& u) const {
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
  double GetLatAccelerationLeftMax() const {
    return acceleration_limits_.lat_acc_left_max;
  }
  double GetLatAccelerationRightMax() const {
    return acceleration_limits_.lat_acc_right_max;
  }
  double GetLonAccelerationMax(const State& x) const {
    return acceleration_limits_.lon_acc_max;
  }
  double GetLonAccelerationMin(const State& x) const {
    // Do not allow to drive backwards
    if (std::abs(x(StateDefinition::VEL_POSITION)) < 1e-5) {
      return 0.0;
    } else {
      return acceleration_limits_.lon_acc_min;
    }
  }

  void SetAccelerationLimits(const AccelerationLimits& acc_lim) {
    acceleration_limits_ = acc_lim;
  }

 private:
  double wheel_base_;
  double steering_angle_max_;
  AccelerationLimits acceleration_limits_;
};

using SingleTrackModelPtr = std::shared_ptr<SingleTrackModel>;

inline double CalculateLateralAcceleration(const SingleTrackModelPtr& model,
                                           const double delta,
                                           const double vel) {
  // Implemented after M. Althoff "CommonRoad: Vehicle Models (Version 2019b)"
  double theta_dot = vel * tan(delta) / model->GetWheelBase();
  double acc_lat = vel * theta_dot;
  return acc_lat;
}

inline double CalculateSteeringAngle(const SingleTrackModelPtr& model,
                                     const State& state,
                                     const bark::geometry::Line& ref_line,
                                     double gain, bool limit_steering = true) {
  // Implemented after G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, and S.
  // Thrun, “Autonomous Automobile Trajectory Tracking for Off-Road Driving:
  // Controller Design, Experimental Validation and Racing,” in 2007 ACC
  //
  // Author: Luis Gressenbuch
  using bark::commons::transformation::FrenetState;
  using StateDefinition::THETA_POSITION;
  using StateDefinition::X_POSITION;
  using StateDefinition::Y_POSITION;
  using bark::geometry::Point2d;
  using bark::geometry::NormToPI;

  const double l = model->GetWheelBase();

  // Calculating State of Front Axle
  State state_front(static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  state_front = state;
  state_front(X_POSITION) = state(X_POSITION) + l * cos(state(THETA_POSITION));
  state_front(Y_POSITION) = state(Y_POSITION) + l * sin(state(THETA_POSITION));

  FrenetState f_state = FrenetState(state_front, ref_line);
  double vel = state(StateDefinition::VEL_POSITION);
  double delta = -NormToPI(f_state.angle) + atan2(-gain * f_state.lat, vel);

  VLOG(5) << "del=" << delta << ", fa=" << f_state.angle << ", na=" << NormToPI(f_state.angle) <<
              ", g=" << gain << ", lat=" << f_state.lat << ", v=" << vel;

  if (limit_steering) {
    double delta_max_right =
        std::min(model->GetSteeringAngleMax(),
                 std::abs(std::atan2(model->GetLatAccelerationRightMax() *
                                         model->GetWheelBase(),
                                     vel * vel)));
    double delta_max_left =
        std::min(model->GetSteeringAngleMax(),
                 std::abs(std::atan2(
                     model->GetLatAccelerationLeftMax() * model->GetWheelBase(),
                     vel * vel)));
    double clamped_delta = std::min(delta, delta_max_left);
    clamped_delta = std::max(clamped_delta, -delta_max_right);
    return clamped_delta;
  }
  return delta;
}

}  // namespace dynamic
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
