# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from bark.runtime.scenario import Scenario
from bark.runtime.scenario.scenario_generation import ScenarioGeneration
from bark.runtime.scenario.interaction_dataset_processing import InteractionDatasetReader
from bark.runtime.scenario.interaction_dataset_processing import DatasetDecomposer
from bark.runtime.commons.model_json_conversion import ModelJsonConversion
from bark.runtime.commons import ParameterServer
# PyBind imports
from bark.core.world.map import *
from bark.core.models.dynamic import *
from bark.core.models.execution import *
from bark.core.world.opendrive import *
from bark.core.world.goal_definition import *
from bark.core.geometry import *

import logging


class InteractionDatasetScenarioGenerationFull(ScenarioGeneration):
    # This class reads in a track file from the interaction dataset
    # and generates a scenario for each agent as the eval agent.

    def __init__(self, params=None, num_scenarios=None, random_seed=None):
        self._map_interface = None
        super().__init__(params, num_scenarios, random_seed)

    def initialize_params(self, params):
        super().initialize_params(params)
        params_temp = \
            self._params["Scenario"]["Generation"]["InteractionDatasetScenarioGenerationFull"]
        self._map_file_name = params_temp["MapFilename",
                                          "Path to the open drive map",
                                          "bark/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr"]
        self._track_file_name_list = params_temp["TrackFilenameList",
                                                 "List of Paths to track files (csv)",
                                                 ["bark/runtime/tests/data/interaction_dataset_DEU_Merging_dummy_track.csv"]]
        self._behavior_model = params_temp["BehaviorModel",
                                           "Overwrite static trajectory of others with behavior model", None]
        self._xy_offset = params_temp["XYOffset",
                                      "offset in x and y direction.", [0, 0]]
        self._excluded_tracks = params_temp[
            "ExcludeTracks", "Track IDs to be excluded from the scenario generation", []]
        self._base_params_json = params_temp[
            "BaseParams", "Initial parameters of each scenario", ParameterServer(log_if_default=True)].ConvertToDict()
        self._agent_params = []
        self._starting_offset_ms = params_temp["StartingOffsetMs",
                                               "Starting Offset to each agent in miliseconds", 500]
        self._road_ids = params_temp["RoadIds",
                                     "Road ids for road corridor.", [0, 1]]
        self._vehicle_length_max = params_temp["VehicleLengthMax",
                                               "Maximum allowed vehicle length", 5.0]
        self._use_shape_from_track = params_temp["UseShapeFromTrack",
                                                 "Use shape from track", True]
        self._use_rectangle_shape = params_temp["RectangleShape",
                                                "Use Rectangle vehicle shape", True]
        self._use_goal_from_road = params_temp["GoalFromRoad",
                                               "Use goal from road", False]
        self._rel_pose_goal_on_road = params_temp["RelPoseGoalOnRoad",
                                                  "Relative position of goal on road", 0.99]

        self._interaction_ds_reader = InteractionDatasetReader(
            use_shape_from_track=self._use_shape_from_track, use_rectangle_shape=self._use_rectangle_shape)

    # TODO: remove code duplication with configurable scenario generation
    def create_scenarios(self, params, num_scenarios):
        """
            see baseclass
        """
        scenario_list = []

        self._map_interface = self.__create_map_interface__()
        self._road_corridor = self.__create_road_corridor__()

        for track_file_name in self._track_file_name_list:

            dataset_decomposer = DatasetDecomposer(map_interface=self._map_interface,
                                                   road_corridor=self._road_corridor,
                                                   track_filename=track_file_name,
                                                   vehicle_length_max=self._vehicle_length_max,
                                                   xy_offset=self._xy_offset,
                                                   starting_offset_ms=self._starting_offset_ms)
            scenario_track_info_list = dataset_decomposer.decompose()

            num = min(num_scenarios, len(scenario_track_info_list))

            for idx_s, sti in enumerate(scenario_track_info_list):
                if idx_s < num_scenarios and sti.GetEgoTrackInfo().GetTrackId() not in self._excluded_tracks:

                    logging.info("Creating scenario {}/{}".format(idx_s, num))
                    try:
                        scenario = self.__create_single_scenario__(sti)
                    except:
                        raise ValueError(
                            "Generation of scenario failed: {}".format(sti))
                    scenario_list.append(scenario)
                else:
                    break
        return scenario_list

    def __create_single_scenario__(self, scen_track_info):
        scen_track_info.TimeSanityCheck()

        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=self._params.ConvertToDict(),
                            map_interface=self._map_interface)

        world = scenario.GetWorldState()
        track_params = ParameterServer()
        track_params["execution_model"] = 'ExecutionModelInterpolate'
        track_params["dynamic_model"] = 'SingleTrackModel'
        track_params["map_interface"] = world.map

        all_track_ids = list(scen_track_info.GetOtherTrackInfos().keys())
        # also add ego id
        ego_track_id = scen_track_info.GetEgoTrackInfo().GetTrackId()
        all_track_ids.append(ego_track_id)

        agent_list = []
        model_converter = ModelJsonConversion()
        for track_id in all_track_ids:
            if self._behavior_model and track_id != ego_track_id:
                behavior_params = self.__fill_agent_params()
                behavior_model_name = self._behavior_model
                track_params["behavior_model"] = model_converter.convert_model(
                    behavior_model_name, behavior_params)
                # behavior_params.Save("/tmp/agent_prams_{}.json".format(track_id))
            else:
                # we do not change behavior model of ego agent -> will be set in benchmark
                track_params["behavior_model"] = None

            if self._use_goal_from_road:
                goal = self.__infer_goal_from_road__(lc=0)
            else:
                goal = None

            agent = self._interaction_ds_reader.AgentFromTrackfile(
                track_params, self._params, scen_track_info, track_id, goal_def=goal)

            # set first valid time stamp of the agent (in relation to scenario start)
            agent.first_valid_timestamp = scen_track_info.GetTimeOffsetOfAgentInSec(
                track_id)

            agent_list.append(agent)

        scenario._agent_list = agent_list  # must contain all agents!
        scenario._eval_agent_ids = [ego_track_id]
        scenario.json_params["track_file"] = scen_track_info.GetTrackFilename()

        return scenario

    def __fill_agent_params(self):
        agent_params = ParameterServer(
            log_if_default=True, json=self._base_params_json)
        self._agent_params.append(agent_params)
        # print("\n", agent_params.ConvertToDict())
        return agent_params

    def __create_map_interface__(self):
        params = ParameterServer()
        # we are creating a dummy scenario to get the map interface from it
        scenario = Scenario(map_file_name=self._map_file_name,
                            json_params=params.ConvertToDict())
        world = scenario.GetWorldState()
        map_interface = world.map

        return map_interface

    def __create_road_corridor__(self):
        if self._map_interface is not None:
            map_interface = self._map_interface
            map_interface.GenerateRoadCorridor(
                self._road_ids, XodrDrivingDirection.forward)
        else:
            raise ValueError("map interface not available")
        return map_interface.GetRoadCorridor(self._road_ids, XodrDrivingDirection.forward)

    def __infer_goal_from_road__(self, lc):
        lane_corr = self._road_corridor.lane_corridors[0]
        goal_polygon = Polygon2d([0, 0, 0], [
                                 Point2d(-10, -10), Point2d(-10, 10), Point2d(10, 10), Point2d(10, -10)])
        goal_point = GetPointAtS(
            lane_corr.center_line, lane_corr.center_line.Length()*self._rel_pose_goal_on_road)
        goal_polygon = goal_polygon.Translate(goal_point)
        return GoalDefinitionPolygon(goal_polygon)
