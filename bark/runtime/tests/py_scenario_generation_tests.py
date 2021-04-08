# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

try:
  import debug_settings
except:
  print("No debugging")

import unittest
import os
from bark.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration

from bark.runtime.scenario.scenario_generation.configurable_scenario_generation \
  import ConfigurableScenarioGeneration, add_config_reader_module

from bark.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation \
    import InteractionDatasetScenarioGeneration
from bark.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation_full \
    import InteractionDatasetScenarioGenerationFull
from bark.runtime.commons import ParameterServer

from bark.core.geometry import *
from bark.core.world.agent import Agent
from bark.core.models.behavior import *
import os
from bark.core.world.evaluation import EvaluatorCollisionEgoAgent, EvaluatorCollisionAgents

from bark.core.commons import SetVerboseLevel

class ScenarioGenerationTests(unittest.TestCase):
  def test_configurable_scenario_generation_default_params(self):
    params = ParameterServer()
    mapfile = os.path.join(os.path.dirname(__file__),"data/city_highway_straight.xodr")
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["MapFilename"] = mapfile
    scenario_generation = ConfigurableScenarioGeneration(
        num_scenarios=2, params=params)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(
        scenario_generation._scenario_list[0]._agent_list))

    scenario = scenario_loader.get_scenario(idx=0)
    params.Save("default_params.json")

  def test_configurable_scenario_generation_sample_behavior_types(self):
    sink_source_dict = [{
        "SourceSink": [[-1.057, -172.1695],  [-1.894, 14.1725]],
        "Description": "left_lane",
        "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [0]},
        "ConfigBehaviorModels": {"Type": "FixedBehaviorType", "ModelType": "BehaviorIDMClassic", "ModelParams":  {"BehaviorIDMClassic::MaxVelocity": 60.0}},
        "ConfigExecutionModels": {"Type": "FixedExecutionType"},
        "ConfigDynamicModels": {"Type": "FixedDynamicType"},
        "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
        "ConfigControlledAgents": {"Type": "NoneControlled"},
        "AgentParams": {}
    },
        {
        "SourceSink": [[-1.057, -172.1695],  [-1.894, 14.1725]],
        "Description": "right_lane",
        "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [1]},
        "ConfigBehaviorModels": {"Type": "SampleBehaviorType"},
        "ConfigExecutionModels": {"Type": "FixedExecutionType"},
        "ConfigDynamicModels": {"Type": "FixedDynamicType"},
        "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
        "ConfigControlledAgents": {"Type": "RandomSingleAgent"},
        "AgentParams": {}
        }]
    params = ParameterServer()
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = sink_source_dict
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["MapFilename"] = \
      os.path.join(os.path.dirname(__file__),"data/city_highway_straight.xodr")
    scenario_generation = ConfigurableScenarioGeneration(
        num_scenarios=2, params=params)
    scenario_generation.dump_scenario_list("test.scenario")
    params.Save("default_params_behavior_type_sampling.json")

  def test_configurable_scenario_generation_interaction_merging_track_ids(self):
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track.csv")
    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    sink_source_dict = {
      "SourceSink": [[1001.92, 1005.59],  [883.064, 1009.07] ],
      "Description": "merging_deu_standard",
      "ConfigAgentStatesGeometries": {"Type": "InteractionDataTrackIdsStatesGeometries", "TrackFilename": track_filename},
      "ConfigBehaviorModels": {"Type": "InteractionDataBehaviors"},
      "ConfigExecutionModels": {"Type": "FixedExecutionType"},
      "ConfigDynamicModels": {"Type": "FixedDynamicType"},
      "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
      "ConfigControlledAgents": {"Type": "AgentIds", "ControlledIds" : [1]},
      "AgentParams" : {}
    }

    params = ParameterServer()
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = [sink_source_dict]
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["MapFilename"] = map_filename
    scenario_generation = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

    scenario = scenario_loader.get_scenario(idx=0)

    params.Save("default_params_interaction_dataset.json")

  def test_configurable_scenario_generation_interaction_merging_window(self):
    track_filenames =  os.path.join(os.path.dirname(__file__), "data/*_dataset_DEU_Merging_dummy_track_late.csv")
    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    sink_source_dict = {
      "SourceSink": [[1001.92, 1005.59],  [883.064, 1009.07] ],
      "Description": "merging_deu_standard",
      "ConfigAgentStatesGeometries": {"Type": "InteractionDataWindowStatesGeometries", "TrackFilenames": track_filenames},
      "ConfigBehaviorModels": {"Type": "InteractionDataBehaviors"},
      "ConfigExecutionModels": {"Type": "FixedExecutionType"},
      "ConfigDynamicModels": {"Type": "FixedDynamicType"},
      "ConfigGoalDefinitions": {"Type": "FixedGoalTypes", "EnforceControlledGoal": False},
      "ConfigControlledAgents": {"Type": "PositioningSingleAgent", "LanePosition" : 0},
      "AgentParams" : {}
    }

    params = ParameterServer()
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = [sink_source_dict]
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["MapFilename"] = map_filename
    scenario_generation = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

    scenario = scenario_loader.get_scenario(idx=0)

    # test if window is reset
    scenario_generation2 = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    self.assertEqual(len(scenario_generation2._scenario_list), 2)

    params.Save("default_params_interaction_dataset.json")

  def test_configurable_scenario_generation_add_module_dir(self):
    sink_source_dict = {
      "SourceSink": [[1001.92, 1005.59],  [883.064, 1009.07] ],
      "Description": "merging_deu_standard",
      "ConfigAgentStatesGeometries": {"Type": "UniformVehicleDistribution", "LanePositions": [0]},
      "ConfigBehaviorModels": {"Type": "TestReaderFixedBehaviorType"},
      "ConfigExecutionModels": {"Type": "FixedExecutionType"},
      "ConfigDynamicModels": {"Type": "FixedDynamicType"},
      "ConfigGoalDefinitions": {"Type": "FixedGoalTypes"},
      "ConfigControlledAgents": {"Type": "RandomSingleAgent"},
      "AgentParams" : {}
    }

    params = ParameterServer()
    mapfile = os.path.join(os.path.dirname(__file__),"data/city_highway_straight.xodr")
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["MapFilename"] = mapfile
    params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]["SinksSources"] = [sink_source_dict]
    add_config_reader_module("bark.runtime.tests.test_config_reader_module")
    scenario_generation = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

    scenario = scenario_loader.get_scenario(idx=0)

    # test if window is reset
    scenario_generation2 = ConfigurableScenarioGeneration(num_scenarios=2,params=params)
    self.assertEqual(len(scenario_generation2._scenario_list), 2)

    params.Save("default_params_interaction_dataset.json")


  def test_find_overlaps_configurable_scenario_generation(self):
    shape = Polygon2d([0, 0, 0], [Point2d(-1,0),
                      Point2d(-1,1),
                      Point2d(1,1),
                      Point2d(1,0)])

    agent_states1 = [[0, 1, 0, 0, 0], [0, 4, 0, 0, 0], [0, 8, 0, 0, 0]] # agents along x axis
    agent_geometries1 = [shape, shape, shape]

    agent_states2 = [[0, 4, -10, 0, 0], [0, 4, 0, 0, 0], [0, 4, 20, 0, 0]] # agents along y axis at x= 4
    agent_geometries2 = [shape, shape, shape]

    agent_states3 = [[0, 20, -20, 0, 0], [0, 1, 0, 0, 0], [0, 4, 20, 0, 0]] # some agents two colliding with other configs
    agent_geometries3 = [shape, shape, shape]

    collected_sources_sinks_agent_states_geometries = [(agent_states1, agent_geometries1),
                                                  (agent_states2, agent_geometries2),
                                                  (agent_states3, agent_geometries3)]

    overlaps = ConfigurableScenarioGeneration.find_overlaps_in_sources_sinks_agents(
                  collected_sources_sinks_agent_states_geometries)

    self.assertTrue("0-1" in overlaps)

    collisions_01 = overlaps["0-1"]
    self.assertEqual(len(collisions_01), 1)

    # check source sink configs
    self.assertEqual(collisions_01[0][0][0], 0)
    self.assertEqual(collisions_01[0][1][0], 1)

    # check agent positions in list
    self.assertEqual(collisions_01[0][0][1], 1)
    self.assertEqual(collisions_01[0][1][1], 1)

    self.assertTrue("0-2" in overlaps)

    collisions_02 = overlaps["0-2"]
    self.assertEqual(len(collisions_02), 1)

    # check source sink configs
    self.assertEqual(collisions_02[0][0][0], 0)
    self.assertEqual(collisions_02[0][1][0], 2)

    # check agent positions in list
    self.assertEqual(collisions_02[0][0][1], 0)
    self.assertEqual(collisions_02[0][1][1], 1)

    collisions_03 = overlaps["1-2"]
    self.assertEqual(len(collisions_03), 1)

    # check source sink configs
    self.assertEqual(collisions_03[0][0][0], 1)
    self.assertEqual(collisions_03[0][1][0], 2)

    # check agent positions in list
    self.assertEqual(collisions_03[0][0][1], 2)
    self.assertEqual(collisions_03[0][1][1], 2)


  def test_configurable_scenario_generation_parametric_observer(self):
    params = ParameterServer()
    mapfile = os.path.join(os.path.dirname(__file__),"data/city_highway_straight.xodr")
    conf_params = params["Scenario"]["Generation"]["ConfigurableScenarioGeneration"]
    conf_params["MapFilename"] = mapfile
    conf_params["ObserverModel"] = {
      "Description": "world_observer",
      "ConfigObserverModel": {
        "Type": "ObserverModelParametricReader"
      }
    }
    scenario_generation = ConfigurableScenarioGeneration(
      num_scenarios=2, params=params)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(
      scenario_generation._scenario_list[0]._agent_list))

    scenario = scenario_loader.get_scenario(idx=0)
    params.Save("default_params_observer.json")


  def test_dataset_scenario_generation_full(self):
    """
    Checking Track file with two track ids, both become valid at time 0. 
    Two Scenarios should be created from this.
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=2)

    self.assertEqual(scenario_generation.get_num_scenarios(), 2)

    # first scenario
    scenario0 = scenario_generation.get_scenario(0)
    agent11 = scenario0.GetWorldState().agents[1]
    agent12 = scenario0.GetWorldState().agents[2]
    self.assertEqual(agent11.first_valid_timestamp, 0.0)
    self.assertEqual(agent12.first_valid_timestamp, 0.0)
    # agents are initialized with Behavior::NotValidYet
    self.assertEqual(list(scenario0.GetWorldState().agents_valid.keys()), [])
    scenario0.GetWorldState().Step(0.01)
    self.assertEqual(list(scenario0.GetWorldState().agents_valid.keys()), [1,2])

    # second scenario
    scenario1 = scenario_generation.get_scenario(1)
    agent21 = scenario1.GetWorldState().agents[1]
    agent22 = scenario1.GetWorldState().agents[2]
    self.assertEqual(agent21.first_valid_timestamp, 0.0)
    self.assertEqual(agent22.first_valid_timestamp, 0.0)
    self.assertEqual(list(scenario1.GetWorldState().agents_valid.keys()), [])
    scenario1.GetWorldState().Step(0.01)
    self.assertEqual(list(scenario1.GetWorldState().agents_valid.keys()), [1,2])

  def test_dataset_scenario_generation_full_incomplete(self):
    """
    Checking Track file with three track ids, but agent 1 is never inside the map, 
    so it should only generate 2 scenarios
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_CHN_Merging_ZS_partial_v02.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_CHN_Merging_dummy_track_incomplete.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=3)
    self.assertEqual(scenario_generation.get_num_scenarios(), 2)

    # first scenario
    agent12 = scenario_generation.get_scenario(0).GetWorldState().agents[2]
    agent17 = scenario_generation.get_scenario(0).GetWorldState().agents[7]
    self.assertEqual(agent12.first_valid_timestamp, 0.0)
    self.assertEqual(agent17.first_valid_timestamp, 0.0)

    # second scenario
    agent22 = scenario_generation.get_scenario(1).GetWorldState().agents[2]
    agent27 = scenario_generation.get_scenario(1).GetWorldState().agents[7]
    self.assertEqual(agent22.first_valid_timestamp, 0.0)
    self.assertEqual(agent27.first_valid_timestamp, 0.0)

  def test_dataset_scenario_generation_full_late(self):
    """
    Checking Track file with two track ids. test wether agent 2 coming in late is 
    correctly identified as invalid at first world time step.
    """
    
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track_late.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=1)

    scenario = scenario_generation.get_scenario(0)
    world_state = scenario.GetWorldState()
    agent1 = world_state.GetAgent(1)
    agent2 = world_state.GetAgent(2)

    self.assertAlmostEqual(agent1.first_valid_timestamp, 0.0)
    self.assertAlmostEqual(agent2.first_valid_timestamp, 0.3)
    
    self.assertEqual(isinstance(agent1, Agent), True)
    self.assertEqual(agent1.IsValidAtTime(world_state.time), True)
    
    self.assertEqual(isinstance(agent2, Agent), True)
    self.assertEqual(agent2.IsValidAtTime(world_state.time), False)

  def test_dataset_scenario_generation_full_late_behavior_overwritten(self):
    """
    Checking Track file with two track ids. test wether agent 2 coming in late is 
    correctly identified as invalid at first world time step. Behavior Model of others
    is overwritten
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track_late.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=1)

    scenario = scenario_generation.get_scenario(0)
    world_state = scenario.GetWorldState()
    agent1 = world_state.GetAgent(1)
    agent2 = world_state.GetAgent(2)

    self.assertAlmostEqual(agent1.first_valid_timestamp, 0.0)
    self.assertAlmostEqual(agent2.first_valid_timestamp, 0.3)
    
    self.assertEqual(isinstance(agent1, Agent), True)
    self.assertEqual(agent1.IsValidAtTime(world_state.time), True)
    
    self.assertEqual(isinstance(agent2, Agent), True)
    self.assertEqual(agent2.IsValidAtTime(world_state.time), False)

  def test_dataset_scenario_generation_full_outside1(self):
    """
    Checking Track file with three track ids. test wether agent 3 (not the ego agent 
    of the scenario) outside at the beginning is correctly identified as invalid at first 
    world time step, but becomes valid later.
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track_outside.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=1)

    scenario = scenario_generation.get_scenario(0)
    self.assertAlmostEqual(scenario.eval_agent_ids, [1])

    world_state = scenario.GetWorldState()
    agent11 = world_state.GetAgent(1)
    agent12 = world_state.GetAgent(2)
    agent13 = world_state.GetAgent(3)

    self.assertAlmostEqual(agent11.first_valid_timestamp, 0.0)
    self.assertAlmostEqual(agent12.first_valid_timestamp, 0.0)
    self.assertNotEqual(agent13.first_valid_timestamp, 0.0)
    
    # agent13 should not be valid at the beginning, as he is outside of map
    world_state.time = 0
    self.assertEqual(isinstance(agent11, Agent), True)
    self.assertEqual(agent11.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent11.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent12, Agent), True)
    self.assertEqual(agent12.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent12.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent13, Agent), True)
    self.assertEqual(agent13.IsValidAtTime(world_state.time), False)
    # as we use only state once it's in map, this will be true, although the time step is not valid yet
    self.assertEqual(agent13.InsideRoadCorridor(), True)

    # agent13 should not be valid at the beginning, as he is outside of map
    world_state.Step(0.05)

    self.assertEqual(isinstance(agent11, Agent), True)
    self.assertEqual(agent11.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent11.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent12, Agent), True)
    self.assertEqual(agent12.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent12.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent13, Agent), True)
    self.assertEqual(agent13.IsValidAtTime(world_state.time), False)
    # as we use only state once it's in map, this will be true, although the time step is not valid yet
    self.assertEqual(agent13.InsideRoadCorridor(), True)

    self.assertEqual(list(world_state.agents_valid.keys()), [1,2])

    # agent13 should be valid at some point
    world_state.Step(agent13.first_valid_timestamp)
    world_state.Step(0.01) # agent13.IsValidAtTime() uses previous time stamp, therefore we increment it one more step

    self.assertEqual(isinstance(agent11, Agent), True)
    self.assertEqual(agent11.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent11.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent12, Agent), True)
    self.assertEqual(agent12.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent12.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent13, Agent), True)
    self.assertEqual(agent13.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent13.InsideRoadCorridor(), True)

    self.assertEqual(list(world_state.agents_valid.keys()), [1,2,3])


  def test_dataset_scenario_generation_full_outside1_behavior_overwritten(self):
    """
    Checking Track file with three track ids. test wether agent 3 (not the ego agent 
    of the scenario) outside at the beginning is correctly identified as invalid at first 
    world time step, but becomes valid later. Overwriting Behavior Model.
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track_outside.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["BehaviorModel"] = "BehaviorMobilRuleBased"

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=1)

    scenario = scenario_generation.get_scenario(0)
    self.assertAlmostEqual(scenario.eval_agent_ids, [1])

    world_state = scenario.GetWorldState()
    agent11 = world_state.GetAgent(1)
    agent12 = world_state.GetAgent(2)
    agent13 = world_state.GetAgent(3)

    # others are defined as BehaviorMobilRuleBased
    self.assertTrue(isinstance(agent11.behavior_model, BehaviorStaticTrajectory))
    self.assertTrue(isinstance(agent12.behavior_model, BehaviorMobilRuleBased))
    self.assertTrue(isinstance(agent13.behavior_model, BehaviorMobilRuleBased))

    self.assertAlmostEqual(agent11.first_valid_timestamp, 0.0)
    self.assertAlmostEqual(agent12.first_valid_timestamp, 0.0)
    self.assertNotEqual(agent13.first_valid_timestamp, 0.0)
    
    # agent13 should not be valid at the beginning, as he is outside of map
    world_state.time = 0
    self.assertEqual(isinstance(agent11, Agent), True)
    self.assertEqual(agent11.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent11.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent12, Agent), True)
    self.assertEqual(agent12.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent12.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent13, Agent), True)
    self.assertEqual(agent13.IsValidAtTime(world_state.time), False)
    # as we use only state once it's in map, this will be true, although the time step is not valid yet
    self.assertEqual(agent13.InsideRoadCorridor(), True)

    # agent13 should not be valid at the beginning, as he is outside of map
    world_state.Step(0.05)

    self.assertEqual(isinstance(agent11, Agent), True)
    self.assertEqual(agent11.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent11.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent12, Agent), True)
    self.assertEqual(agent12.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent12.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent13, Agent), True)
    self.assertEqual(agent13.IsValidAtTime(world_state.time), False)
    # as we use only state once it's in map, this will be true, although the time step is not valid yet
    self.assertEqual(agent13.InsideRoadCorridor(), True)

    self.assertEqual(list(world_state.agents_valid.keys()), [1,2])

    # agent13 should be valid at some point
    world_state.Step(agent13.first_valid_timestamp)
    world_state.Step(0.01) # agent13.IsValidAtTime() uses previous time stamp, therefore we increment it one more step

    self.assertEqual(isinstance(agent11, Agent), True)
    self.assertEqual(agent11.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent11.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent12, Agent), True)
    self.assertEqual(agent12.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent12.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent13, Agent), True)
    self.assertEqual(agent13.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent13.InsideRoadCorridor(), True)

    self.assertEqual(list(world_state.agents_valid.keys()), [1,2,3])

  def test_dataset_scenario_generation_full_outside3(self):
    """
    Checking Track file with three track ids. test wether agent 3 (the ego agent 
    of the scenario) outside at the beginning is correctly identified as valid at first 
    world time step (although in the track files, it becomes valid later than the other agents 
    -> will be cut off for scenario)
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track_outside.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=3)

    scenario = scenario_generation.get_scenario(2)
    self.assertAlmostEqual(scenario.eval_agent_ids, [3])
    world_state = scenario.GetWorldState()
    agent31 = world_state.GetAgent(1)
    agent32 = world_state.GetAgent(2)
    agent33 = world_state.GetAgent(3)

    # they all should be valid at the beginning
    world_state.time = 0
    self.assertEqual(isinstance(agent31, Agent), True)
    self.assertEqual(agent31.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent31.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent32, Agent), True)
    self.assertEqual(agent32.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent32.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent33, Agent), True)
    self.assertEqual(agent33.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent33.InsideRoadCorridor(), True)

    world_state.Step(0.05)
    self.assertEqual(len(world_state.agents_valid), 3)

  def test_dataset_scenario_generation_full_outside3_behavior_overwritten(self):
    """
    Checking Track file with three track ids. test wether agent 3 (the ego agent 
    of the scenario) outside at the beginning is correctly identified as valid at first 
    world time step (although in the track files, it becomes valid later than the other agents 
    -> will be cut off for scenario). Overwriting Behavior Model.
    """
    params = ParameterServer()

    map_filename =  os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename =  os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track_outside.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["TrackFilenameList"] = [track_filename]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["StartingOffsetMs"] = 0
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]["BehaviorModel"] = "BehaviorMobilRuleBased"

    scenario_generation = InteractionDatasetScenarioGenerationFull(
        params=params, num_scenarios=3)

    scenario = scenario_generation.get_scenario(2)
    self.assertAlmostEqual(scenario.eval_agent_ids, [3])
    world_state = scenario.GetWorldState()
    agent31 = world_state.GetAgent(1)
    agent32 = world_state.GetAgent(2)
    agent33 = world_state.GetAgent(3)

    # others are defined as BehaviorMobilRuleBased
    self.assertTrue(isinstance(agent31.behavior_model, BehaviorMobilRuleBased))
    self.assertTrue(isinstance(agent32.behavior_model, BehaviorMobilRuleBased))
    self.assertTrue(isinstance(agent33.behavior_model, BehaviorStaticTrajectory))

    # they all should be valid at the beginning
    world_state.time = 0
    self.assertEqual(isinstance(agent31, Agent), True)
    self.assertEqual(agent31.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent31.InsideRoadCorridor(), True)
    
    self.assertEqual(isinstance(agent32, Agent), True)
    self.assertEqual(agent32.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent32.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent33, Agent), True)
    self.assertEqual(agent33.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent33.InsideRoadCorridor(), True)

    self.assertEqual(isinstance(agent33, Agent), True)
    self.assertEqual(agent33.IsValidAtTime(world_state.time), True)
    self.assertEqual(agent33.InsideRoadCorridor(), True)

    evaluator = EvaluatorCollisionAgents()
    world_state.AddEvaluator("collision", evaluator)
    info = world_state.Evaluate()
    self.assertEqual(info["collision"], False)

    world_state.Step(0.05)

    evaluator = EvaluatorCollisionAgents()
    world_state.AddEvaluator("collision", evaluator)
    info = world_state.Evaluate()
    self.assertEqual(info["collision"], False)

    self.assertEqual(len(world_state.agents_valid), 3)

  def test_dataset_scenario_generation(self):
    params = ParameterServer()

    map_filename = os.path.join(os.path.dirname(__file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename = os.path.join(os.path.dirname(__file__), "data/interaction_dataset_DEU_Merging_dummy_track.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["TrackFilename"] = track_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["TrackIds"] = [
        1, 2]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["StartTs"] = 500
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["EndTs"] = 1000
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["EgoTrackId"] = 1

    scenario_generation = InteractionDatasetScenarioGeneration(
        params=params, num_scenarios=1)
    self.assertEqual(scenario_generation.get_num_scenarios(), 1)

  def test_dataset_scenario_generation_overwrite_behavior_model(self):
    params = ParameterServer()

    map_filename = os.path.join(os.path.dirname(
        __file__), "data/DR_DEU_Merging_MT_v01_shifted.xodr")
    track_filename = os.path.join(os.path.dirname(
        __file__), "data/interaction_dataset_DEU_Merging_dummy_track.csv")

    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["MapFilename"] = map_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["TrackFilename"] = track_filename
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["TrackIds"] = [
        1, 2]
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["StartTs"] = 500
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["EndTs"] = 1000
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["EgoTrackId"] = 1
    params["Scenario"]["Generation"]["InteractionDatasetScenarioGeneration"]["BehaviorModel"] = {
        "2": "BehaviorMobilRuleBased"}

    scenario_generation = InteractionDatasetScenarioGeneration(
        params=params, num_scenarios=1)

    beh_model = scenario_generation.get_scenario(0)._agent_list[1].behavior_model
    self.assertTrue(isinstance(beh_model, BehaviorMobilRuleBased))

if __name__ == '__main__':
  SetVerboseLevel(2)
  unittest.main()
