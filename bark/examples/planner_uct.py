# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


from bark.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from bark.runtime.commons import ParameterServer
from bark.runtime.viewer import MPViewer
from bark.runtime.viewer import VideoRenderer
import os
behavior_used = None
try:
  from bark.pybark.core.models.behavior import BehaviorUCTSingleAgent
  behavior_used = BehaviorUCTSingleAgent
except:
  print("BehaviorUCTSingleAgent not available, rerun example with `bazel run //bark/examples:planner_uct --define planner_uct=true ")
  exit()

scenario_param_file ="uct_planner.json" # must be within examples params folder
param_server = ParameterServer(filename= os.path.join(os.path.dirname(__file__),"params",scenario_param_file))

scenario_generation = UniformVehicleDistribution(num_scenarios=1, random_seed=0, params=param_server)


viewer = MPViewer(params=param_server, x_range=[5060, 5160], y_range=[5070,5150])
sim_step_time = param_server["simulation"]["step_time",
                                        "Step-time used in simulation",
                                        0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                "execution in real-time or faster", 1]
scenario, idx = scenario_generation.get_next_scenario()

world_state = scenario.get_world_state()
world_state.agents[scenario._eval_agent_ids[0]].behavior_model = BehaviorUCTSingleAgent(param_server)
param_server.save("bark/examples/params/mcts_params_written.json")

# world_state.agents[scenario._eval_agent_ids[0]].behavior_model

video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)
for _ in range(0, 40): # run scenario for 100 steps
  world_state.DoPlanning(sim_step_time)
  video_renderer.drawWorld(world_state, scenario._eval_agent_ids)
  world_state.DoExecution(sim_step_time)

video_renderer.export_video(filename=os.path.join(os.path.dirname(__file__),"scenarios/test_video_intermediate"), remove_image_dir=True)
