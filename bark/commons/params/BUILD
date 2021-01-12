package(default_visibility = ["//visibility:public"])

cc_library(
    name = "params",
    hdrs = [
        "params.hpp",
        "params_test.h",
        "setter_params.hpp"
    ],
    srcs = [
        "setter_params.cpp",
        "params.cpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/commons/util:util",
        "//bark/commons/distribution:distribution",
        "@boost//:geometry", # add since sometimes "boost/variant.hpp" not found
    ]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libcorec.so",
#   srcs = glob([
#     #   "commons/params/*.hpp",
#     #   "commons/params/*.cpp",
#     "*.cpp", "*.hpp",
#     # "commons/*.cpp", "commons/*.hpp",
#     # "geometry/*.cpp", "geometry/*.hpp",
#   #  "world/*.cpp", "world/*.hpp",
#   #  "models/*.cpp", "models/*.hpp",
#   #  "viewer/*.cpp", "viewer/*.hpp",
#   #  "runtime.hpp"
#     ]),
    srcs = [
        "setter_params.cpp",
        "params.cpp",
        "params.hpp",
        "params_test.h",
        "setter_params.hpp"
    ],
    deps = [
        "//bark/commons/util:util",
        "//bark/commons/distribution:distribution",
        "@boost//:geometry", # add since sometimes "boost/variant.hpp" not found
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
