# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.scenario_generation.scenario_generation import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.model_json_conversion import ModelJsonConversion
from bark.world.agent import *
from bark.models.behavior import *
from bark.world import *
from bark.world.map import *
from bark.models.dynamic import *
from bark.models.execution import *
from bark.geometry import *
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.roadgraph_generator import RoadgraphGenerator
from modules.runtime.commons.xodr_parser import XodrParser

import numpy as np


class UniformVehicleDistribution(ScenarioGeneration):
    def __init__(self, num_scenarios, params=None, random_seed=None):
        super(UniformVehicleDistribution, self).__init__(params, num_scenarios, random_seed)
        self.initialize_params(params)

    def initialize_params(self, params):
        params_temp = self.params["Scenario"]["Generation"]["UniformVehicleDistribution"]

        self.map_file_name = params_temp["MapFilename", "Path to the open drive map", 
                     "modules/runtime/tests/data/Crossing8Course.xodr"]
        self.ego_source = params_temp["EgoSource", "A point around which the ego agent spawns. A lane must be near this point (<0.5m) \
                         Provide x,y coordinates as list", [-11,-8] ]
        self.others_source = params_temp["OthersSource", "A list of points around which other vehicles spawn. \
                                         Points should be on different lanes. Lanes must be near these points (<0.5m) \
                                         Provide a list of lists with x,y-coordinates", [[-16.626,-14.8305],[-162.325, -77.0077]]  ]
        self.others_sink = params_temp["OthersSink", "A list of points around which other vehicles are deleted. \
                                        Points should be on different lanes and match the order of the source points. \
                                        Lanes must be near these points (<0.5m) \
                                        Provide a list of lists with x,y-coordinates", [[-191.789,-50.1725],[-13.61,-19.4288]]  ]   
        assert(len(self.others_sink), len(self.others_source))            

        self.vehicle_distance_range = params_temp["VehicleDistanceRange", "Distance range between vehicles in meter given as tuple from which distances are sampled uniformly", (15, 20)]
        self.velocity_range = params_temp["VehicleVelocityRange", "Lower and upper bound of velocity in km/h given as tuple from which velocities are sampled uniformly", (20,30)]

        json_converter = ModelJsonConversion()
        self.agent_params = params_temp["VehicleModel", "How to model the agent", \
             json_converter.agent_to_json(self.default_agent_model())]

        np.random.seed(self.random_seed)


    def create_scenarios(self, params, num_scenarios, random_seed):
        """ 
            see baseclass
        """
        scenario_list = []
        for scenario_idx in range(0, num_scenarios):
            scenario = self.create_single_scenario()     
            scenario_list.append(scenario)

        return scenario_list

    def create_single_scenario(self):
        
        world = World(self.params)
        self.setup_map(world, self.map_file_name)

        for idx, source in enumerate(self.others_source):
            connecting_center_line, s_start, s_end, _, lane_id_end = \
                     self.center_line_between_source_and_sink( world.map,  source, self.others_sink[idx])
            self.place_agents_along_linestring(world, connecting_center_line, s_start, s_end, \
                                                                             self.agent_params, lane_id_end)

        description=self.params.convert_to_dict()
        description["ScenarioGenerator"] = "UniformVehicleDistribution"
        scenario = Scenario(world_state=world, description={"ScenarioGenerator": "UniformVehicleDistribution"})
        return scenario

    def place_agents_along_linestring(self, world, linestring, s_start, s_end, agent_params, goal_lane_id):
        s = s_start
        if s_end < s_start:
            linestring.reverse()
            s = s_end
            s_end = s_start

        while s < s_end:
            # set agent state on linestring with random velocity
            xy_point =  get_point_at_s(linestring, s)
            angle = get_tangent_angle_at_s(linestring, s)
            
            velocity = self.sample_velocity_uniform(self.velocity_range)
            agent_state = np.array([0, xy_point.x(), xy_point.y(), angle, velocity ])

            agent_params = self.agent_params.copy()
            agent_params["state"] = agent_state
            agent_params["goal_lane_id"] = goal_lane_id
            agent_params["map_interface"] = world.map

            converter = ModelJsonConversion()
            bark_agent = converter.agent_from_json(agent_params, self.params)
            world.add_agent(bark_agent)

            # move forward on linestring based on vehicle size and max/min distance
            s += bark_agent.shape.front_dist + bark_agent.shape.rear_dist + \
                        self.sample_distance_uniform(self.vehicle_distance_range)




    def sample_velocity_uniform(self, velocity_range):
        return np.random.uniform(velocity_range[0], velocity_range[1])

    def sample_distance_uniform(self, distance_range):
        return np.random.uniform(distance_range[0], distance_range[1])


    def center_line_between_source_and_sink(self, map_interface, source, sink):
        lane_source = map_interface.find_nearest_lanes(Point2d(source[0],source[1]),1)[0]
        lane_sink = map_interface.find_nearest_lanes(Point2d(sink[0],sink[1]),1)[0]
        left_line, right_line, center_line = map_interface.calculate_driving_corridor(lane_source.lane_id, lane_sink.lane_id)

        _, s_start, _ = get_nearest_point_and_s(center_line, Point2d(source[0],source[1]))
        _, s_end, _ = get_nearest_point_and_s(center_line, Point2d(sink[0],sink[1]))
        return center_line, s_start, s_end, lane_source.lane_id, lane_sink.lane_id

    def setup_map(self, world, map_file_name):
        xodr_parser = XodrParser(map_file_name )
        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        world.set_map(map_interface)


    def default_agent_model(self):
        param_server = ParameterServer()
        behavior_model = BehaviorConstantVelocity(param_server)
        execution_model = ExecutionModelInterpolate(param_server)
        dynamic_model = SingleTrackModel()
        map_interface = MapInterface()

        agent_2d_shape = CarLimousine()
        init_state = np.array([0, 0, 0, 0, 0])

        agent_default = Agent(init_state,
                    behavior_model,
                    dynamic_model,
                    execution_model,
                    agent_2d_shape,
                    param_server,
                    2,
                    None)

        return agent_default



    



    


