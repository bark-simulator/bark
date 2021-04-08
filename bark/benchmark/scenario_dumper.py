# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from pathlib import Path
import os
from datetime import datetime
import pandas as pd
import numpy as np
import logging
from shutil import copyfile
import pathlib
import ntpath
from bark.benchmark.benchmark_analyzer import BenchmarkAnalyzer
from bark.runtime.viewer.video_renderer import VideoRenderer
from bark.runtime.viewer import MPViewer
from bark.runtime.commons.parameters import ParameterServer
from lxml import etree
from pathlib import Path
from bark.core.models.dynamic import StateDefinition

class ScenarioDumper(BenchmarkAnalyzer):
  def __init__(self, base_result_folder, benchmark_result):
    super(ScenarioDumper, self).__init__(benchmark_result)

    # Create result dir
    if not os.path.isdir(base_result_folder):
      Path(base_result_folder).mkdir(parents=True, exist_ok=True)
    this_result_folder = datetime.now().strftime("%d%m%Y_%H:%M:%S")
    scenario_behavior = self._benchmark_result.get_benchmark_configs()[0].scenario_set_name \
      + "_" + self._benchmark_result.get_benchmark_configs()[0].behavior_config.behavior_name
    self._result_folder = os.path.join(base_result_folder, scenario_behavior, this_result_folder)
    Path(self._result_folder).mkdir(parents=True, exist_ok=True)

  # Based on the given filter dump the matching scenarios
  # @note we do not check if the entries in config_idx_list really exist!
  def export_scenarios_filter(self, filter = {}, config_idx_list = [], export_video=False):
    if filter:
      configs_found = super().find_configs(filter)
    else:
      configs_found = []

    if configs_found and config_idx_list:
      configs_found = list(set(configs_found) & set(config_idx_list))
    elif not configs_found and config_idx_list:
      configs_found = config_idx_list
    elif configs_found and not config_idx_list:
      # noting to do
      configs_found = configs_found
    else: #both empty -> error
      raise ValueError("Either specify a non-empty filter of a valid list of indices!")

    for config in configs_found:
      self.export(config, export_video)

    return configs_found

  # Dump a scenario given by the index in the result
  def export(self, config_idx, export_video=False):
    this_folder = os.path.join(self._result_folder, "ConfigIdx_"+str(config_idx))
    Path(this_folder).mkdir(parents=True, exist_ok=True)
    self.write_trajectory(config_idx, this_folder)
    self.write_scenario_parameter(config_idx, this_folder)
    self.write_behavior_parameter(config_idx, this_folder)
    self.write_map(config_idx, self._result_folder)
    if export_video:
      self.render_video(config_idx, this_folder)

  # Write video
  def render_video(self, config_idx, folder):
    viewer = MPViewer(
      params=ParameterServer(),
      center= [375, 0],
      enforce_x_length=True,
      x_length = 100.0,
      use_world_bounds=True)
    video_exporter = VideoRenderer(renderer=viewer, world_step_time=0.2)
    super().visualize(viewer=video_exporter,  configs_idx_list=[config_idx], \
                  real_time_factor=10, fontsize=6)
    video_exporter.export_video(filename=os.path.join(folder,"video"), \
      remove_image_dir = True)

  # Collect all points from the trajectories of all agents. 
  # Note we use the historic states of the last world
  def write_trajectory(self, config_idx, folder):
    cols = ['angent_id','time','x','y','theta','v']
    table = []
    histories = super().get_benchmark_result().get_history(config_idx)
    if histories is None:
      logging.warning("No historic state saved, cannot dump trajetory")
      return
    scenario = histories[-1] #the last state inclues all the historic states
    world = scenario.GetWorldState()
    for (agent_id, agent) in world.agents.items():
      for state_action in agent.history:
        state = state_action[0]
        table.append([agent_id, state[0], state[1], state[2], state[3], state[4]])
    np_table = np.array(table)
    df = pd.DataFrame(np_table, columns=cols)
    df.to_csv(os.path.join(folder, "trajectories.csv"), sep='\t', encoding='utf-8')
    #print(df.to_string())

  def write_scenario_parameter(self, config_idx, folder):
    benchmark_config = super().get_benchmark_result().get_benchmark_config(config_idx)
    if benchmark_config is not None:
      params = benchmark_config.scenario.json_params
      p = ParameterServer()
      p.ConvertToParam(params)
      p.Save(os.path.join(folder, "scenario_parameters.json"))

  def write_behavior_parameter(self, config_idx, folder):
    scenarios = super().get_benchmark_result().get_history(config_idx)
    if scenarios is not None:
      scenario = scenarios[-1]
      world = scenario.GetWorldState()
      params = world.GetParams()
      # TODO das muss doch gehen... aus einem Params object einen ParameterServer erstellen. aber ich sehe nicht wie.
      #p = ParameterServer(params)
      #p.Save(os.path.join(folder, "behavior_parameters.json"))
      #print(p)

  def write_map(self, config_idx, folder):
    benchmark_config = super().get_benchmark_result().get_benchmark_config(config_idx)
    if benchmark_config is not None:
      params = benchmark_config.scenario.json_params
      mapfile = params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["MapFilename"]
      src = os.path.join("src", "database", mapfile)
      copyfile(src, os.path.join(folder, ntpath.basename(mapfile)))


class OpenScenarioDumper(ScenarioDumper):
  def __init__(self, base_result_folder, benchmark_result):
    super(OpenScenarioDumper, self).__init__(
      base_result_folder, benchmark_result)
  
  def GetTrajectoryPerAgent(self, world):
    trajectory_per_agents = {}
    for (agent_id, agent) in world.agents.items():
      trajectory_per_agents[agent_id] = []
      for state_action in agent.history:
        state = state_action[0]
        trajectory_per_agents[agent_id].append(state)
    return trajectory_per_agents
  
  def GetTemplates(self):
    curr_folder = pathlib.Path(__file__).parent.absolute()
    template_xml = str(curr_folder) + "/templates/template_traj.xml"
    with open(template_xml, 'r') as f:
      xml_file = etree.parse(f)
    vertex_template = xml_file.findall('.//Vertex')[0]
    temp_vertex = etree.tostring(vertex_template)
    vertex_template.getparent().remove(vertex_template)
    temp_xml = etree.tostring(xml_file)
    return temp_vertex, temp_xml
    
  def write_trajectory(self, config_idx, folder):
    histories = super().get_benchmark_result().get_history(config_idx)
    if histories is None:
      logging.warning("No historic state saved, cannot dump trajetory.")
      return
    # one history for each time-step
    total_history_length = len(histories)
    scenario = histories[-1]  # the last state inclues all the historic states
    world = scenario.GetWorldState()
    trajectory_per_agents = self.GetTrajectoryPerAgent(world)
    temp_vertex, temp_xml = self.GetTemplates()
    
    for agent_id, traj in trajectory_per_agents.items():
      if len(traj) < total_history_length:
        logging.warning("State history in agent does not cover all time-step.")
      agent_xml = etree.fromstring(temp_xml)
      polyline = agent_xml.findall(".//Polyline")[0]
      for state in traj:
        vertex = etree.fromstring(temp_vertex)
        vertex.attrib['time'] = str(state[int(StateDefinition.TIME_POSITION)])
        world_pos = vertex.find("Position").find("WorldPosition")
        world_pos.attrib['x'] = str(state[int(StateDefinition.X_POSITION)])
        world_pos.attrib['y'] = str(state[int(StateDefinition.Y_POSITION)])
        world_pos.attrib['z'] = str(0.)
        world_pos.attrib['h'] = str(state[int(StateDefinition.THETA_POSITION)])
        polyline.append(vertex)
      filename = os.path.join(folder, "trajectory_" + str(agent_id) + ".xml")
      et = etree.ElementTree(agent_xml)
      et.write(filename, pretty_print=True)
