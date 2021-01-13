
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
        "//bark/python_wrapper/tests:py_glog_init",
        "//bark/models/tests:py_behavior_model_test",
        "//bark/geometry/tests:py_geometry_tests",
        "//bark/commons/tests:py_commons_tests",
        "//bark/benchmark/tests:py_benchmark_runner_tests",
        "//bark/benchmark/tests:py_benchmark_result_tests",
        "//bark/benchmark/tests:py_benchmark_process_tests",
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
            "//bark/examples:scenarios",
            "//bark/examples:params"
            ],
  deps = [":tests",
        "//bark/runtime/viewer:threaded_viewer",
        "//bark/runtime/viewer:panda3d_easy",
        "//bark/runtime/tests:test_config_reader_module",
        "//bark/runtime/scenario/scenario_generation:scenario_generation",
        "//bark/runtime/scenario/scenario_generation/config_readers:config_readers",
        "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
        "//bark/runtime/commons:xodr_parser",
        "//bark/runtime/commons:parameters",
        "//bark/runtime/commons:model_json_conversion",
        "//bark/models/behavior/plan:py_planners",
        "//bark/benchmark:benchmark_runner_mp",
        "//bark/benchmark:benchmark_runner",
        "//bark/benchmark:benchmark_analyzer",
        "//bark/benchmark:benchmark_result",
        "//bark/runtime:runtime",
        "//bark/runtime/viewer:video_renderer",
        "//bark/runtime/viewer:pygame_viewer",
        "//bark/runtime/viewer:panda3d_viewer",
        "//bark/runtime/viewer:matplotlib_viewer",
        "//bark/runtime/viewer:py_viewer",
        "//bark/runtime/scenario:scenario",
        "//bark/runtime/commons:commons",
        "//bark/examples:paths",
        "//bark/examples:highway",
        "//bark/examples:highway_rss",
        "//bark/examples:merging",
        "//bark/examples:merging_rss",
        "//bark/examples:intersection",
        "//bark/examples:maude_interface",
        "//bark/examples:maude",
        "//bark/examples:interaction_dataset"
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

# For apollo integration
cc_binary(
    name = "libcorec.so",
    srcs = [
        # NOTE the cpp files have to be included here, otherwise the symbols are not in the lib. only having the files in the dependencies is not enough
        "//bark/commons/distribution:distribution.hpp",
        "//bark/commons/params:setter_params.cpp",
        "//bark/commons/params:params.cpp",
        "//bark/commons/params:params.hpp",
        "//bark/commons/params:setter_params.hpp",
#        "//bark/world/opendrive:opendrive.hpp",
    #    "//bark/world/map:road_corridor.hpp",
    #    "//bark/world/map:road_corridor.cpp",
    #    "//bark/world/map:lane_corridor.hpp",
    #    "//bark/world/map:lane_corridor.cpp",
    ],
    deps = [
        "//bark/commons/distribution:distribution",
        "//bark/commons/params:params",
        "//bark/world/map:map_interface",
    #    "//bark/world/map:road_corridor",
    #    "//bark/world/map:lane_corridor",
        "//bark/commons/util:util",
    #    "//bark/commons/distribution:distribution",
    #    "@boost//:geometry", # add since sometimes "boost/variant.hpp" not found
    # "@com_github_eigen_eigen//:eigen",
    # "//bark/geometry:geometry",
    # "//bark/world/opendrive:opendrive",
    # "//bark/world/tests:make_test_xodr_map",
    # "//bark/world/tests:make_test_world",
    # "//bark/world/map:roadgraph",
    # "//bark/world/goal_definition:goal_definition",
    # "//bark/world/evaluation:evaluation",
    # "//bark/world/evaluation/ltl/label_functions:label_function",
    # "//bark/world/evaluation/ltl:evaluator_ltl",
    #"//bark/commons/params:params",
    # "//bark/commons/util:util",
    # "//bark/models/execution/interpolation:interpolation",
    # "//bark/models/behavior/constant_acceleration:constant_acceleration",
    # "//bark/models/behavior/motion_primitives:motion_primitives",
    # "//bark/models/behavior/dynamic_model:dynamic_model",
    # "//bark/models/behavior/idm:idm_classic",
    # "//bark/models/behavior/idm:idm_lane_tracking",
    # "//bark/models/behavior/rule_based:lane_change_behavior",
    # "//bark/models/behavior/rule_based:intersection_behavior",
    # "//bark/models/behavior/rule_based:mobil_behavior",
    # "//bark/models/behavior/static_trajectory",
    # "//bark/models/behavior/not_started",
    # "//bark/models/behavior/idm/stochastic:stochastic",
    # "//bark/models/behavior/behavior_safety:behavior_safety",
    # "//bark/models/behavior/behavior_rss:behavior_rss",
    # #"//bark/models/execution/mpc:mpc",
    # "//bark/runtime/viewer:viewer",
    # "//bark/world:world",
    # "//bark/runtime:cc_runtime",
    # "//bark/world/evaluation/rss:safety_polygon"
    ],
    #   ] + select({"//bark/models/behavior/plan:_planner_uct" : ["@planner_uct//bark_mcts/python_wrapper:planner_uct"], "//conditions:default": []})
    #   + select({"//bark/models/behavior/plan:_planner_rules_mcts" : ["@planner_rules_mcts//python:planner_rules_mcts"], "//conditions:default": []})
    #   + select({"//bark/world/evaluation/ltl:_ltl_rules": ["@rule_monitor_project//python/bindings:rule_monitor"], "//conditions:default": []}),
    #   copts = select({"//bark/models/behavior/plan:_planner_uct" : ["-D PLANNER_UCT"], "//conditions:default": []})
    #   + select({"//bark/models/behavior/plan:_planner_rules_mcts" : ["-D PLANNER_RULES_MCTS"], "//conditions:default": []})
    #   + select({"//bark/world/evaluation/ltl:_ltl_rules" : ["-D LTL_RULES"], "//conditions:default": []})
    #   + select({"//bark/world/evaluation/rss:_rss" : ["-D RSS"], "//conditions:default": []})
    #   + select({"//bark/models/behavior/plan:_planner_miqp" : ["-D PLANNER_MIQP", "-DIL_STD", "-fPIC", "-DILOUSEMT", "-D_REENTRANT"], "//conditions:default": []}),
    linkshared = 1,
    linkstatic = 1
)