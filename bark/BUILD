
genrule(
  name = "generate_core",
  srcs = [
    "//bark/python_wrapper:core.so",
  ],
  cmd = (" mv $(RULEDIR)/python_wrapper/core.so  $(location core.so)") ,
  #]),
  outs = [
    "core.so",
  ],
  visibility = ["//visibility:public"],
)


# obtained tests using 
#bazel query 'filter(.tests.py, //...)'
py_test (
    name="tests",
    data = [':generate_core'],
    srcs=["tests.py",
        "//bark/world/tests:py_world_tests",
        "//bark/world/tests:py_system_tests",
        "//bark/world/tests/map:py_roadgraph_test",
        "//bark/world/tests/map:py_road_corridor_tests",
        "//bark/world/tests/map:py_opendrive_tests",
        "//bark/world/tests/map:py_map_interface_tests",
        "//bark/world/tests/objects:py_agent_tests",
        "//bark/runtime/tests:py_scenario_generation_tests",
        "//bark/runtime/tests:py_param_server_tests",
        "//bark/runtime/tests:py_interaction_dataset_reader_test",
        "//bark/runtime/tests:py_interaction_dataset_decomposer_test",
        "//bark/runtime/tests:py_importer_tests",
        "//bark/runtime/tests:py_evaluation_tests",
        "//bark/python_wrapper/tests:py_pickle_tests",
        "//bark/models/tests:py_behavior_model_test",
        "//bark/geometry/tests:py_geometry_tests",
        "//bark/commons/tests:py_commons_tests",
        "//bark/benchmark/tests:py_benchmark_runner_tests",
       # "//bark/benchmark/tests:py_benchmark_process_tests",
        "//bark/benchmark/tests:py_benchmark_analyzer_tests"
    ],
)


# Add py_libraries and data
# bazel query 'kind("py_library",//...)' for deps
# bazel query 'kind("filegroup",//...)' for data
py_test(
  name = "pip_package",
  srcs = ["pip_package.py", "__init__.py"],
  data = [":generate_core",
            "//docs/tutorials:notebooks_folder",
            "//bark/world/tests:params2",
            "//bark/runtime/viewer:model_data",
            "//bark/runtime/tests:xodr_data",
            "//bark/runtime/tests:track_data",
            ],
  deps = [":tests",
        "//bark/runtime/viewer:threaded_viewer",
        "//bark/runtime/viewer:panda3d_easy",
        "//bark/runtime/scenario/scenario_generation:scenario_generation",
        "//bark/runtime/scenario/scenario_generation/config_readers:config_readers",
        "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
        "//bark/runtime/commons:xodr_parser",
        "//bark/runtime/commons:parameters",
        "//bark/runtime/commons:model_json_conversion",
        "//bark/models/behavior/plan:py_planners",
        "//bark/benchmark:benchmark_runner_mp",
        "//bark/benchmark:benchmark_runner",
        "//bark/runtime:runtime",
        "//bark/runtime/viewer:video_renderer",
        "//bark/runtime/viewer:pygame_viewer",
        "//bark/runtime/viewer:panda3d_viewer",
        "//bark/runtime/viewer:matplotlib_viewer",
        "//bark/runtime/viewer:py_viewer",
        "//bark/runtime/scenario:scenario",
        "//bark/runtime/commons:commons",
        "//bark/benchmark:benchmark_analyzer",
       
          ],

)

cc_library(
    name="include",
    deps=[
        "//bark/commons:include",
        "//bark/geometry:include",
        "//bark/models:include",
        "//bark/world:include"
    ],
    visibility = ["//visibility:public"],
)
