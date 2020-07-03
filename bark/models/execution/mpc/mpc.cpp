// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <iostream>
#include <vector>

#include "bark/models/execution/mpc/mpc.hpp"

namespace bark {
namespace models {

// TODO(@all) !
execution::ExecutionModelMpc::ExecutionModelMpc(
    const bark::commons::ParamsPtr params)
    : ExecutionModel(params) {
  optimization_settings_.num_optimization_steps = params->GetInt(
      "NumOptimizationSteps",
      "Number of optimization steps for Execution Optimimizer", 20);
  optimization_settings_.dt = params->GetReal(
      "OptimizationStepSize", "Step Size for Execution Optimimizer", 0.1);
}

dynamic::Trajectory execution::ExecutionModelMpc::Execute(
    const float& new_world_time, const dynamic::Trajectory& trajectory,
    const dynamic::DynamicModelPtr dynamic_model,
    const dynamic::State current_state) {
  double current_world_time = current_state(StateDefinition::TIME_POSITION);

  // initial estimate of atomic element
  std::vector<double> acceleration, steering_rate;

  for (int i = 0; i < optimization_settings_.num_optimization_steps; i++) {
    acceleration.push_back(0.0);
    steering_rate.push_back(0.0);
  }

  // make parameter_block compatible with ceres-format
  std::vector<double*> parameter_block;
  parameter_block.push_back(&acceleration[0]);
  parameter_block.push_back(&steering_rate[0]);

  dynamic::Trajectory desired_states(
      optimization_settings_.num_optimization_steps,
      static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  desired_states.setZero();
  desired_states.col(StateDefinition::TIME_POSITION) =
      Eigen::ArrayXf::LinSpaced(
          optimization_settings_.num_optimization_steps, current_world_time,
          current_world_time +
              (optimization_settings_.num_optimization_steps - 1) *
                  optimization_settings_.dt);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> weights_desired_states(
      optimization_settings_.num_optimization_steps,
      static_cast<int>(StateDefinition::MIN_STATE_SIZE));
  weights_desired_states.setZero();
  weights_desired_states.col(StateDefinition::TIME_POSITION) =
      Eigen::ArrayXf::LinSpaced(
          optimization_settings_.num_optimization_steps, current_world_time,
          current_world_time +
              (optimization_settings_.num_optimization_steps - 1) *
                  optimization_settings_.dt)
          .cast<double>();

  // get discrete states ready for optimization grid
  for (int i = 0; i < trajectory.rows(); ++i) {
    for (int j = 0; j < optimization_settings_.num_optimization_steps; j++) {
      if (trajectory(i, StateDefinition::TIME_POSITION) ==
          current_world_time + j * optimization_settings_.dt) {
        desired_states.row(j) = trajectory.row(i);

        // set weights
        weights_desired_states(j, StateDefinition::X_POSITION) = 100;
        weights_desired_states(j, StateDefinition::Y_POSITION) = 100;
        weights_desired_states(j, StateDefinition::THETA_POSITION) = 100;
        weights_desired_states(j, StateDefinition::VEL_POSITION) = 0;

        break;
      }
    }
  }

  // Overwrite first desired entry with current vehicle state no matter what
  // planner wants std::cout << "Current State \n" << current_state <<
  // std::endl;

  desired_states.row(0) = current_state;
  weights_desired_states(0, StateDefinition::X_POSITION) = 100;
  weights_desired_states(0, StateDefinition::Y_POSITION) = 100;
  weights_desired_states(0, StateDefinition::THETA_POSITION) = 100;
  weights_desired_states(0, StateDefinition::VEL_POSITION) = 0;

  dynamic::Trajectory optimized_trajectory =
      Optimize(parameter_block, desired_states, weights_desired_states);

  // safe optimized_trajectory and weights for debugging
  SetLastTrajectory(optimized_trajectory);
  set_last_desired_states(desired_states);
  set_last_weights(weights_desired_states);

  return optimized_trajectory;
}

dynamic::Trajectory execution::ExecutionModelMpc::Optimize(
    std::vector<double*> parameter_block,
    const dynamic::Trajectory& discrete_states,
    const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
        weights_desired_states) {
  // objective function
  CostFunctor* cost_functor = new CostFunctor();
  cost_functor->set_desired_states(discrete_states);
  cost_functor->set_optimization_settings(optimization_settings_);
  cost_functor->set_weights(weights_desired_states);

  // Build the problem.
  ceres::Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  ceres::DynamicAutoDiffCostFunction<
      CostFunctor, static_cast<int>(StateDefinition::MIN_STATE_SIZE)>*
      cost_function = new ceres::DynamicAutoDiffCostFunction<
          CostFunctor, static_cast<int>(StateDefinition::MIN_STATE_SIZE)>(
          cost_functor);

  cost_function->AddParameterBlock(
      optimization_settings_.num_optimization_steps - 1);
  cost_function->AddParameterBlock(
      optimization_settings_.num_optimization_steps - 1);

  cost_function->SetNumResiduals(1);

  problem.AddResidualBlock(cost_function, new ceres::TrivialLoss(),
                           parameter_block);

  // Run the solver!
  ceres::Solver::Options options;
  // options.max_num_consecutive_invalid_steps = 1000;
  options.max_num_iterations = 4000;
  options.minimizer_type = ceres::MinimizerType::LINE_SEARCH;
  options.function_tolerance = 10e-8;
  options.line_search_direction_type = ceres::LineSearchDirectionType::LBFGS;

  options.use_inner_iterations = false;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";

  ceres::Problem::EvaluateOptions eval_options;
  double total_cost = 0.0;
  std::vector<double> residuals;
  problem.Evaluate(eval_options, &total_cost, &residuals, nullptr, nullptr);

  const double* const* const_parameter_block;
  const_parameter_block =
      reinterpret_cast<const double* const*>(&parameter_block[0]);

  // call kinematic model one last time to get optimized trajectory
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> traj =
      KinematicModel<double>(const_parameter_block, discrete_states,
                             optimization_settings_);

  return traj.cast<float>();
}

}  // namespace models
}  // namespace bark
