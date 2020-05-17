import os
import matplotlib.pyplot as plt
from IPython.display import Video

from load.benchmark_database import BenchmarkDatabase
from serialization.database_serializer import DatabaseSerializer
from modules.benchmark.benchmark_runner import BenchmarkRunner, BenchmarkConfig, BenchmarkResult
from modules.benchmark.benchmark_analyzer import BenchmarkAnalyzer

from modules.runtime.commons.parameters import ParameterServer

from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.video_renderer import VideoRenderer


from bark.models.behavior import BehaviorIDMClassic, BehaviorConstantVelocity

#dbs = DatabaseSerializer(test_scenarios=4, test_world_steps=5, num_serialize_scenarios=10)
dbs = DatabaseSerializer(test_scenarios=1, test_world_steps=10, num_serialize_scenarios=1)
dbs.process("../../../benchmark_database/data/database1")
local_release_filename = dbs.release(version="tutorial")

print('Filename:', local_release_filename)