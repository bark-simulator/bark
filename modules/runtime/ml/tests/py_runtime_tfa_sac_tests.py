# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.ml.runtime_rl import RuntimeRL
from modules.runtime.ml.tfa_wrapper import TFAWrapper
from modules.runtime.ml.nn_state_observer import StateConcatenation
from modules.runtime.ml.action_wrapper import MotionPrimitives, DynamicModel
from modules.runtime.ml.state_evaluator import GoalReached
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer

# tfa
import tensorflow as tf
tf.compat.v1.enable_v2_behavior()
from tf_agents.environments import tf_py_environment
from tf_agents.environments import utils
from tf_agents.networks import actor_distribution_network
from tf_agents.networks import normal_projection_network
from tf_agents.agents.ddpg import critic_network
from tf_agents.policies import greedy_policy
from tf_agents.metrics import tf_metrics
from tf_agents.drivers import dynamic_step_driver
from tf_agents.eval import metric_utils

from tf_agents.environments import tf_py_environment
from tf_agents.agents.sac import sac_agent
from tf_agents.replay_buffers import tf_uniform_replay_buffer
from tf_agents.utils import common
from tf_agents.trajectories import time_step as ts
from tf_agents.utils.common import Checkpointer

# get the distribution to sample our actions from - for SAC only
def normal_projection_net(action_spec, init_means_output_factor=0.1):
  return normal_projection_network.NormalProjectionNetwork(
    action_spec,
    mean_transform=None,
    state_dependent_std=True,
    init_means_output_factor=init_means_output_factor,
    std_transform=sac_agent.std_clip_transform,
    scale_distribution=True)

# Evaluation based on average of rewards
def compute_avg_return(environment, policy, num_episodes=5):
  total_return = 0.0
  for i in range(num_episodes):
    print("================ Episode {} ================".format(str(i)))
    time_step = environment.reset()
    episode_return = 0.0
    steps = 1.0
    while not time_step.is_last():
      action_step = policy.action(time_step)
      print(action_step.action, time_step)
      time_step = environment.step(action_step.action)
      episode_return += time_step.reward
      steps += 1.0
    total_return += episode_return/steps
  avg_return = total_return / num_episodes
  return avg_return.numpy()[0]

class RuntimeSACTests(unittest.TestCase):
  @staticmethod
  def test_motion_primitives_concat_state():
    params = ParameterServer(filename="modules/runtime/tests/data/highway_merging.json")
    scenario_generation = UniformVehicleDistribution(num_scenarios=3, random_seed=0, params=params)
    state_observer = StateConcatenation(params=params)
    action_wrapper = DynamicModel(params=params)
    evaluator = GoalReached(params=params)
    viewer = MPViewer(params=params, x_range=[-30,30], y_range=[-20,40], follow_agent_id=True) #use_world_bounds=True) # 

    runtimerl = RuntimeRL(action_wrapper=action_wrapper, nn_observer=state_observer,
                            evaluator=evaluator, step_time=0.05, viewer=viewer,
                            scenario_generator=scenario_generation)


    train_env = tf_py_environment.TFPyEnvironment(TFAWrapper(runtimerl))
    eval_env = tf_py_environment.TFPyEnvironment(TFAWrapper(runtimerl))
    
    # hyper parameters
    actor_fc_layer_params = (512, 512, 256) # 4 layer net right now # changes 14 from (1024, 512, 512, 256)
    critic_joint_fc_layer_params = (256, 256, 128) # 2 layer critic net
    actor_learning_rate = 1e-4  # @param
    critic_learning_rate = 2e-4   # @param changes 14 from m
    alpha_learning_rate = 3e-4 # @param
    target_update_tau = 0.005 # @param # changed in changes 14 from 0.005
    target_update_period = 1 #@param changes in changes 14 from 1
    reward_scale_factor = 1.0 #@param

    initial_collect_steps = 1000  # @param
    gamma = 0.99 #@param
    gradient_clipping = None # @param
    replay_buffer_capacity = 1000000 # @param
    collect_episodes_per_iteration = 10
    num_iterations = 40000
    num_eval_episodes = 50
    eval_interval = 500  # @param - at how many steps should we perform the evaluation metrics
    collect_steps_per_iteration = 1
    batch_size = 256  # @param
    log_interval = 10  # @param - how often do we want to have our milestones printed
    global_step = tf.compat.v1.train.get_or_create_global_step()
    agent_name = "SAC_agent"

    # Actor will predict Normal dist paramters, where we sample actions from
    actor_net = actor_distribution_network.ActorDistributionNetwork(
        train_env.observation_spec(),
        train_env.action_spec(),
        fc_layer_params=actor_fc_layer_params,
        continuous_projection_net=normal_projection_net)

    # Critic Network to get Q(s,a) - estimate how good action is in state
    critic_net = critic_network.CriticNetwork(
        #(env.observation_space.shape, env.action_space.shape),
        (train_env.observation_spec(), train_env.action_spec()),
        observation_fc_layer_params=None,
        action_fc_layer_params=None,
        joint_fc_layer_params=critic_joint_fc_layer_params)

    # Create SAC agent
    tf_agent = sac_agent.SacAgent(
        train_env.time_step_spec(),
        train_env.action_spec(),
        actor_network=actor_net,
        critic_network=critic_net,
        actor_optimizer=tf.compat.v1.train.AdamOptimizer(
            learning_rate=actor_learning_rate),
        critic_optimizer=tf.compat.v1.train.AdamOptimizer(
            learning_rate=critic_learning_rate),
        alpha_optimizer=tf.compat.v1.train.AdamOptimizer(
            learning_rate=alpha_learning_rate),
        target_update_tau=target_update_tau,
        target_update_period=target_update_period,
        td_errors_loss_fn=tf.compat.v1.losses.mean_squared_error,
        gamma=gamma,
        reward_scale_factor=reward_scale_factor,
        gradient_clipping=gradient_clipping,
        train_step_counter=global_step,
        name=agent_name)
    tf_agent.initialize()

    #log_path = FLAGS.base_path + "src/checkpoints/sac/"

    # checkpoints
    #checkpointer = Checkpointer(log_path,
    #                            global_step=global_step,
    #                            tf_agent=tf_agent,
    #                            max_to_keep=2)
    #checkpointer.initialize_or_restore()

    # policies
    eval_policy = greedy_policy.GreedyPolicy(tf_agent.policy) # main policy
    collect_policy = tf_agent.collect_policy # behavioral policy (= data collection)
    # training
    replay_buffer = tf_uniform_replay_buffer.TFUniformReplayBuffer(
        data_spec=tf_agent.collect_data_spec,
        batch_size=train_env.batch_size,
        max_length=replay_buffer_capacity)
    
    initial_collect_driver = dynamic_step_driver.DynamicStepDriver(
            train_env,
            collect_policy,
            observers=[replay_buffer.add_batch],
            num_steps=initial_collect_steps)
    print("Collecting initial episodes..")
    #initial_collect_driver.run()

    collect_driver = dynamic_step_driver.DynamicStepDriver(
      train_env,
      collect_policy,
      observers=[replay_buffer.add_batch],
      num_steps=collect_steps_per_iteration)

    # (Optional) Optimize by wrapping some of the code in a graph using TF function.
    collect_driver.run = common.function(collect_driver.run)
    tf_agent.train = common.function(tf_agent.train)

    # Dataset generates trajectories with shape [Bx2x...]
    dataset = replay_buffer.as_dataset(
        num_parallel_calls=3, sample_batch_size=batch_size, num_steps=2).prefetch(3)

    iterator = iter(dataset)

    train_env.reset()
    # Reset the train step
    # tf_agent.train_step_counter.assign(0)
    avg_return = compute_avg_return(eval_env, eval_policy, num_eval_episodes)
    """
    print("\n ------------------------- STARTING SAC AGENT TRAINING -------------------------- \n")
    for i in range(num_iterations):
      # 
      # Collect a few steps using collect_policy and save to the replay buffer.
      for _ in range(collect_episodes_per_iteration):
        collect_driver.run()

      # Sample a batch of data from the buffer and update the agent's network.
      experience, unused_info = next(iterator)
      train_loss = tf_agent.train(experience)

      step = tf_agent.train_step_counter.numpy()

      if step % log_interval == 0:
        print('step = {} from {} loss = {}'.format(step, num_iterations, train_loss.loss))

      if step % eval_interval == 0:
        avg_return = compute_avg_return(train_env, eval_policy)
        print("Average return: {}".format(str(avg_return)))

        # save graph
        # checkpointer.save(global_step)
    """
if __name__ == '__main__':
    unittest.main()