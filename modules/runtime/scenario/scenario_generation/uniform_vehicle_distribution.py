from modules.runtime.scenario.scenario import Scenario
from modules.runtime.scenario.model_json_conversion import ModelJsonConversion
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
from modules.runtime.viewer.pygame_viewer import PygameViewer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.commons.xodr_parser import XodrParser


class UniformVehicleDistribution(ScenarioGeneration):
    def __init__(self, num_scenarios, params=None, random_seed=None):
        super(UniformVehicleDistribution, self).__init__(params, num_scenarios, random_seed)
        self.initialize_params(params)

    def initialize_params(self, params):
        if params is None:
            self.params = ParameterServer()
        else:
            self.params = params
        params_temp = self.params["Scenario"]["Generation"]["UniformVehicleDistribution"]

        self.map_file_name = params_temp["MapFilename", "Path to the open drive map", 
                     "modules/runtime/tests/data/Crossing8Course.xodr"]
        self.scenario_region = params_temp["ScenarioRegion", "A region within the scenario \
                        takes place outside of this region no vehicles are spawned. \
                         Provide a upper left and lower right bbox corners ", [[-100,100],[100,-100]] ]

        self.vehicle_min_distance = params_temp["VehicleMinDistance", "Minimum distance between vehicles", 2]
        self.vehicle_max_distance = params_temp["VehicleMinDistance", "Maximum distance between vehicles", 5]

        json_converter = ModelJsonConversion()
        self.agent_ = params_temp["VehicleModel", "How to model the agent", \
             json_converter.agent_to_json(self.default_agent_model())]



    def create_scenarios(self, params, num_scenarios, random_seed):
        """ 
            see baseclass
        """

        for scenario_idx in range(0, num_scenarios):
            scenario = self.create_single_scenario()     
            self.scenario_list.append(scenario)

    def create_single_scenario():
        
        world = World(self.params)
        self.setup_map(world, self.map_file_name)




        description={}
        description["ScenarioGenerator"] = "UniformVehicleDistribution"
        description["GenerationParams"] = params.convert_to_dict()
        scenario = Scenario(world_state=world, description={"ScenarioGenerator": "UniformVehicleDistribution"})



    def setup_map(self, world, map_file_name):
        xodr_parser = XodrParser(map_file_name )
        map_interface = MapInterface()
        map_interface.set_open_drive_map(xodr_parser.map)
        map_interface.set_roadgraph(xodr_parser.roadgraph)
        world.set_map(map_interface)

    def setup_agents(self, world, params):
        pass


    


    def default_agent_model(self, param_server):
        behavior_model = BehaviorConstantVelocity(param_server)
        execution_model = ExecutionModelInterpolate(param_server)
        dynamic_model = SingleTrackModel()
        map_interface = MapInterface()

        agent_2d_shape = CarLimousine()
        init_state = np.array([0, 0, 0, 0, 0])
        param_server = ParameterServer()
        agent_default = Agent(init_state,
                    behavior_model,
                    dynamic_model,
                    execution_model,
                    agent_2d_shape,
                    param_server,
                    2,
                    map_interface)

        return agent_default



    



    


