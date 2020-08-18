// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_BEHAVIOR_IDM_BASE_IDM_HPP_
#define BARK_MODELS_BEHAVIOR_IDM_BASE_IDM_HPP_

#include <memory>
#include <tuple>
#include <utility>

#include "bark/commons/transformation/frenet.hpp"
#include "bark/models/behavior/behavior_model.hpp"
#include "bark/world/observed_world.hpp"
#include "bark/world/world.hpp"

namespace bark {
namespace models {
namespace behavior {

using bark::geometry::Point2d;
using bark::world::ObservedWorld;
using bark::world::map::LaneCorridor;
using bark::world::map::LaneCorridorPtr;

struct IDMRelativeValues {
  double leading_distance;
  double leading_velocity;
  Continuous1DAction ego_acc;
  Continuous1DAction leading_acc;
  bool has_leading_object;
};

// Base class for all IDMs and the const. vel.
// includes all longitudinal acc. functions
class BaseIDM : virtual public BehaviorModel {
 public:
  explicit BaseIDM(const commons::ParamsPtr& params);

  virtual ~BaseIDM() {}

  virtual Trajectory Plan(float delta_time,
                          const ObservedWorld& observed_world);

  double CalcFreeRoadTerm(const double vel_ego) const;

  double CalcInteractionTerm(const double net_distance, const double vel_ego,
                             const double vel_other) const;
  double CalcNetDistance(
      const world::ObservedWorld& observed_world,
      const std::shared_ptr<const world::objects::Agent>& leading_agent) const;

  std::pair<bool, double> GetDistanceToLaneEnding(
      const LaneCorridorPtr& lane_corr, const Point2d& pos) const;

  virtual std::tuple<Trajectory, Action> GenerateTrajectory(
      const world::ObservedWorld& observed_world,
      const LaneCorridorPtr& lane_corr, const IDMRelativeValues& rel_values,
      double delta_time) const = 0;

  double CalcRawIDMAcc(const double& net_distance, const double& vel_ego,
                       const double& vel_other) const;

  double CalcIDMAcc(const double net_distance, const double vel_ego,
                    const double vel_other) const;

  double CalcCAHAcc(const double& net_distance, const double& vel_ego,
                    const double& vel_other, const double& acc_ego,
                    const double& acc_other) const;

  double CalcACCAcc(const double& net_distance, const double& vel_ego,
                    const double& vel_other, const double& acc_ego,
                    const double& acc_other) const;

  virtual std::pair<double, double> GetTotalAcc(
      const world::ObservedWorld& observed_world,
      const IDMRelativeValues& rel_values, double rel_distance,
      double dt) const;

  IDMRelativeValues CalcRelativeValues(
      const world::ObservedWorld& observed_world,
      const LaneCorridorPtr& lane_corr) const;

  //! Getter and Setter
  virtual float GetMinVelocity() const { return param_min_velocity_; }
  virtual float GetMaxVelocity() const { return param_max_velocity_; }
  const double GetDesiredVelocity() const {
    return param_desired_velocity_;
  }  // unit is meter/second
  const float GetMinimumSpacing() const {
    return param_minimum_spacing_;
  }  // unit is meter
  const float GetDesiredTimeHeadway() const {
    return param_desired_time_head_way_;
  }  // unit is seconds
  const float GetMaxAcceleration() const {
    return param_max_acceleration_;
  }  // unit is meter/second^2
  const float GetAccelerationLowerBound() const {
    return param_acceleration_lower_bound_;
  }
  const float GetAccelerationUpperBound() const {
    return param_acceleration_upper_bound_;
  }
  const int GetNumTrajectoryTimePoints() const {
    return num_trajectory_time_points_;
  }
  const float GetComfortableBrakingAcceleration() const {
    return param_comfortable_braking_acceleration_;
  }  // unit is meter/second^2
  const int GetExponent() const { return param_exponent_; }
  const float GetCoolnessFactor() const { return param_coolness_factor_; }
  LaneCorridorPtr GetLaneCorridor() const { return lane_corr_; }
  void SetLaneCorridor(const LaneCorridorPtr& lane_corr) {
    lane_corr_ = lane_corr;
  }

 protected:
  // Parameters
  float param_minimum_spacing_;
  float param_desired_time_head_way_;
  float param_max_acceleration_;
  float param_acceleration_lower_bound_;
  float param_acceleration_upper_bound_;
  float param_desired_velocity_;
  float param_comfortable_braking_acceleration_;
  float param_min_velocity_;
  float param_max_velocity_;
  int param_exponent_;
  int num_trajectory_time_points_;
  LaneCorridorPtr lane_corr_;

  // IDM extension to stop at the end of the LaneCorridor
  bool brake_lane_end_;
  float brake_lane_end_enabled_distance_;
  float brake_lane_end_distance_offset_;

  // constant acceleration heuristic
  // according chapter 11. Car-Following Models based on Driving Strategies
  // in "Traffic Flow Dynamics" by M.Treiber and A.Kesting
  float param_coolness_factor_;
};

}  // namespace behavior
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_BEHAVIOR_IDM_BASE_IDM_HPP_
