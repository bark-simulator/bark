# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

# ffmpeg must be installed and available on command line

import os
import logging
logging.basicConfig()
import matplotlib.pyplot as plt 
import matplotlib.gridspec as gridspec

from modules.runtime.scenario.scenario_generation.interaction_dataset_scenario_generation_full \
    import InteractionDatasetScenarioGenerationFull

from modules.benchmark.benchmark_runner import BenchmarkResult
from modules.benchmark.benchmark_analyzer import BenchmarkAnalyzer

from serialization.database_serializer import DatabaseSerializer

from load.benchmark_database import BenchmarkDatabase

from modules.benchmark.benchmark_runner import BenchmarkRunner

from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.video_renderer import VideoRenderer
import os

sim_step_time = 0.1
num_scenarios = 1
params = ParameterServer()

dbs = DatabaseSerializer(test_scenarios=1, test_world_steps=5, num_serialize_scenarios=num_scenarios)
dbs.process("database")
local_release_filename = dbs.release(version="2020_08_20")

# # reload
db = BenchmarkDatabase(database_root=local_release_filename)

# Run Benchmark
behaviors_tested = {"default" : None}
evaluators = {"collision": "EvaluatorCollisionAgents", "max_steps": "EvaluatorStepCount"}
# terminal_when = {"collision": lambda x: x,"max_steps": lambda x : x>100}
terminal_when = {"max_steps": lambda x : x>100}

benchmark_runner = BenchmarkRunner(benchmark_database=db,
                                   evaluators=evaluators,
                                   terminal_when=terminal_when,
                                   behaviors=behaviors_tested,
                                   num_scenarios=num_scenarios,
                                   log_eval_avg_every=10)

result = benchmark_runner.run(maintain_history=True)

analyzer = BenchmarkAnalyzer(benchmark_result=result)

configs_selected = analyzer.find_configs()

# setting up the viewer
resolution = (1920, 1080)
dpi = 300
fig_env = plt.figure(figsize=(resolution[0] / dpi, resolution[1] / dpi), dpi=dpi)
gs = gridspec.GridSpec(1, 1, left=0.0, right=1, bottom=0, top=0.9)
axis = plt.subplot(gs[0])
params2 = ParameterServer()
params2["Visualization"]["Agents"]["DrawAgentId"] = True
viewer = MPViewer(params=params2, use_world_bounds=True, axis=axis)
video_exporter = VideoRenderer(renderer=viewer, world_step_time=sim_step_time)

analyzer.visualize(viewer = video_exporter, configs_idx_list=configs_selected, \
                  real_time_factor=1, fontsize=6)


video_exporter.export_video(filename="/tmp/dataset")