// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_MODELS_EXECUTION_MPC_COST_FUNCTOR_HPP_
#define BARK_MODELS_EXECUTION_MPC_COST_FUNCTOR_HPP_

#include <Eigen/Dense>
#include <iostream>
#include "bark/models/execution/mpc/common.hpp"

namespace bark {
namespace models {
namespace execution {
using namespace bark::models::dynamic;

template <typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> KinematicModel(
    T const* const* parameters,
    bark::models::dynamic::Trajectory desired_trajectory,
    const OptimizationSettings& optimization_settings) {
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> traj =
      desired_trajectory.cast<T>();

  // TODO: Lf als Parameter
  T Lf = T(2.6);

  for (int i = 1; i < optimization_settings.num_optimization_steps; i++) {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> f_dot(1, 5);

    // fill matrix
    f_dot << T(1),  // (0) t' = 1
        traj(i - 1, StateDefinition::VEL_POSITION) *
            cos(traj(
                i - 1,
                StateDefinition::THETA_POSITION)),  // (1) x_dot = v*cos(psi)
        traj(i - 1, StateDefinition::VEL_POSITION) *
            sin(traj(
                i - 1,
                StateDefinition::THETA_POSITION)),  // (2) y_dot = v*sin(psi)
        tan(parameters[1][i - 1]) / Lf,  // (3) theta_dot = tan(delta)/Lf
        parameters[0][i - 1];            // (4) v_dot = a

    traj.block(i, 0, 1, 5) =
        traj.block(i - 1, 0, 1, 5) + T(optimization_settings.dt) * f_dot;
  }
  return traj;
}

struct CostFunctor {
  template <typename T>
  bool operator()(T const* const* parameters, T* residual) {
    // initialize
    T cost = T(0.0);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> traj = KinematicModel(
        parameters, desired_discrete_states_, optimization_settings_);
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> desired_discrete_states =
        desired_discrete_states_.cast<T>();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> weights =
        weights_.cast<T>();

    // offsets
    for (int i = 0; i < optimization_settings_.num_optimization_steps - 1;
         i++) {
      T xDiff, yDiff, thetaDiff, velDiff;
      xDiff = traj(i, StateDefinition::X_POSITION) -
              desired_discrete_states(i, StateDefinition::X_POSITION);
      yDiff = traj(i, StateDefinition::Y_POSITION) -
              desired_discrete_states(i, StateDefinition::Y_POSITION);
      thetaDiff = traj(i, StateDefinition::THETA_POSITION) -
                  desired_discrete_states(i, StateDefinition::THETA_POSITION);
      velDiff = traj(i, StateDefinition::VEL_POSITION) -
                desired_discrete_states(i, StateDefinition::VEL_POSITION);

      cost += weights(i, StateDefinition::X_POSITION) * xDiff * xDiff;
      cost += weights(i, StateDefinition::Y_POSITION) * yDiff * yDiff;
      cost +=
          weights(i, StateDefinition::THETA_POSITION) * thetaDiff * thetaDiff;
      cost += weights(i, StateDefinition::VEL_POSITION) * velDiff * velDiff;
    }

    // differential values
    // TODO(@all): seperate weights
    T weights_acc = T(1e1);
    T weights_delta = T(1);
    for (int i = 0; i < optimization_settings_.num_optimization_steps - 1;
         i++) {
      cost +=
          weights_acc * parameters[0][i] * parameters[0][i];  // acceleration
      cost += weights_delta * parameters[1][i] * parameters[1][i];  // steering
    }

    residual[0] = cost / (weights.sum() +
                          (weights_acc + weights_delta) *
                              T(optimization_settings_.num_optimization_steps));
    return true;
  }

  bool set_desired_states(
      const bark::models::dynamic::Trajectory& desired_states) {
    desired_discrete_states_ = desired_states;
    return true;
  }

  bool set_optimization_settings(
      const OptimizationSettings& optimization_settings) {
    optimization_settings_ = optimization_settings;
    return true;
  }

  bool set_weights(
      const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> weights) {
    weights_ = weights;
    return true;
  }

  bark::models::dynamic::Trajectory desired_discrete_states_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> weights_;
  OptimizationSettings optimization_settings_;
};

}  // namespace execution
}  // namespace models
}  // namespace bark

#endif  // BARK_MODELS_EXECUTION_MPC_COST_FUNCTOR_HPP_
