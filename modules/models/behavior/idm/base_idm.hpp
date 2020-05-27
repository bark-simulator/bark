// Copyright (c) 2020 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_MODELS_BEHAVIOR_IDM_BASE_IDM_HPP_
#define MODULES_MODELS_BEHAVIOR_IDM_BASE_IDM_HPP_

#include <memory>
#include <tuple>
#include <utility>

#include "modules/commons/transformation/frenet.hpp"
#include "modules/world/world.hpp"
#include "modules/world/observed_world.hpp"
#include "modules/models/behavior/behavior_model.hpp"

namespace modules {
namespace models {
namespace behavior {

using modules::world::map::LaneCorridor;
using modules::world::map::LaneCorridorPtr;
using modules::world::ObservedWorld;


struct IDMRelativeValues {
  double leading_distance;
  double leading_velocity;
  bool has_leading_object;
};

// Base class for all IDMs and the const. vel.
// includes all longitudinal acc. functions
class BaseIDM : public BehaviorModel {
 public:
  explicit BaseIDM(const commons::ParamsPtr& params);

  virtual ~BaseIDM() {}

  virtual Trajectory Plan(
    float delta_time, const ObservedWorld& observed_world);

  double CalcFreeRoadTerm(const double vel_ego) const;

  double CalcInteractionTerm(const double net_distance, const double vel_ego,
                             const double vel_other) const;
  double CalcNetDistance(
      const std::shared_ptr<const world::objects::Agent>& ego_agent,
      const std::shared_ptr<const world::objects::Agent>& leading_agent) const;

  virtual std::tuple<Trajectory, Action> GenerateTrajectory(
    const world::ObservedWorld& observed_world,
    const LaneCorridorPtr& lane_corr,
    const IDMRelativeValues& rel_values,
    double delta_time) const = 0;

  double CalcRawIDMAcc(const double& net_distance,
                       const double& vel_ego,
                       const double& vel_other) const;

  double CalcIDMAcc(const double net_distance,
                    const double vel_ego,
                    const double vel_other) const;

  virtual std::pair<double, double> GetTotalAcc(
    const world::ObservedWorld& observed_world,
    const IDMRelativeValues& rel_values,
    double rel_distance,
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
    return param_minimum_spacing_; }  // unit is meter
  const float GetDesiredTimeHeadway() const {
    return param_desired_time_head_way_; }  // unit is seconds
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
  LaneCorridorPtr GetLaneCorridor() const { return lane_corr_; }
  void SetLaneCorridor(const LaneCorridorPtr& lane_corr) {
    lane_corr_ = lane_corr;
  }

 private:
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
};


}  // namespace behavior
}  // namespace models
}  // namespace modules

#endif  // MODULES_MODELS_BEHAVIOR_IDM_BASE_IDM_HPP_

