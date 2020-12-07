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
  double lat_acc_max;
  double lat_acc_min;
  double lon_acc_max;
  double lon_acc_min;
};

inline std::ostream& operator<<(std::ostream& os,
                                const AccelerationLimits& al) {
  os << "AccelerationLimits = ("
     << " lat_acc_max: " << al.lat_acc_max << ", "
     << " lat_acc_min: " << al.lat_acc_min << ", "
     << " lon_acc_max: " << al.lon_acc_max << ", "
     << " lon_acc_min: " << al.lon_acc_min << ")";
  return os;
}

inline AccelerationLimits AccelerationLimitsFromParamServer(
    const bark::commons::ParamsPtr& params) {
  AccelerationLimits al = AccelerationLimits();
  al.lat_acc_max =
      params->GetReal("DynamicModel::LatAccMax",
                      "Maximum lateral acceleration [m/s^2]", 4.0);
  al.lat_acc_min =
      params->GetReal("DynamicModel::LatAccMin",
                      "Minimum lateral acceleration [m/s^2]", -4.0);
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
  double GetLatAccelerationMin() const {
    return acceleration_limits_.lat_acc_min;
  }
  double GetLatAccelerationMax() const {
    return acceleration_limits_.lat_acc_max;
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
  // Initial Author: Luis Gressenbuch

  const auto boundValue = [](const double val, const double min, const double max) {
    return std::max(std::min(val, max), min);
  };

  const auto SteeringAngle = [](const double acc_lat, const double vel_lon, const double wheelbase) {
    return std::atan2(acc_lat * wheelbase, vel_lon * vel_lon);
  };

  using bark::commons::transformation::FrenetState;
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

  if (limit_steering) {
    double wb = model->GetWheelBase();
    double delta_max_acc = SteeringAngle(model->GetLatAccelerationMax(), vel, wb);
    double delta_min_acc = SteeringAngle(model->GetLatAccelerationMin(), vel, wb);
    VLOG(5) << "DeltaMaxAcc: " << delta_max_acc << ", DeltaMinAcc: " << delta_min_acc << ", LatAccMax: " << model->GetLatAccelerationMax() << ", LatAccMin: " << model->GetLatAccelerationMin();

    double delta1 = boundValue(delta, -model->GetSteeringAngleMax(), model->GetSteeringAngleMax());
    double delta2 = boundValue(delta1, delta_min_acc, delta_max_acc);
    VLOG(5) << "Delta (unbounded): << " << delta << ", Delta (bound angle): << " << delta1 << ", Delta (bound acc): << " << delta2;
    return delta2;
  }
  return delta;
}

}  // namespace dynamic
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_DYNAMIC_SINGLE_TRACK_HPP_
