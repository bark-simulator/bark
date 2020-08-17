package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "core.so",
  srcs = glob([
    "*.cpp", "*.hpp",
    "commons/*.cpp", "commons/*.hpp",
    "geometry/*.cpp", "geometry/*.hpp",
    "world/*.cpp", "world/*.hpp",
    "models/*.cpp", "models/*.hpp",
    "viewer/*.cpp", "viewer/*.hpp",
    "runtime.hpp"]),
  deps = [
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen",
    "//bark/runtime/tests:py_param_server_test_helper",
    "//bark/geometry:geometry",
    "//bark/world/opendrive:opendrive",
    "//bark/world/tests:make_test_xodr_map",
    "//bark/world/tests:make_test_world",
    "//bark/world/map:roadgraph",
    "//bark/world/goal_definition:goal_definition",
    "//bark/world/evaluation:evaluation",
    "//bark/world/evaluation/ltl/label_functions:label_function",
    "//bark/world/evaluation/ltl:evaluator_ltl",
    "//bark/commons/params:params",
    "//bark/commons/util:util",
    "//bark/models/execution/interpolation:interpolation",
    "//bark/models/behavior/constant_acceleration:constant_acceleration",
    "//bark/models/behavior/motion_primitives:motion_primitives",
    "//bark/models/behavior/dynamic_model:dynamic_model",
    "//bark/models/behavior/idm:idm_classic",
    "//bark/models/behavior/idm:idm_lane_tracking",
    "//bark/models/behavior/rule_based:lane_change_behavior",
    "//bark/models/behavior/rule_based:intersection_behavior",
    "//bark/models/behavior/rule_based:mobil_behavior",
    "//bark/models/behavior/static_trajectory",
    "//bark/models/behavior/idm/stochastic:stochastic",
    #"//bark/models/execution/mpc:mpc",
    "//bark/runtime/viewer:viewer",
    "//bark/world:world",
    "//bark/runtime:cc_runtime",
    "//bark/python_wrapper/models/plan:planners",
    "//bark/python_wrapper/tests:logging_tests"
  ] + select({"//bark/models/behavior/plan:_planner_uct" : ["@planner_uct//bark_mcts/python_wrapper:planner_uct"], "//conditions:default": []})
  + select({"//bark/world/evaluation/ltl:_ltl_rules": ["@rule_monitor_project//python/bindings:rule_monitor"], "//conditions:default": []}),
  copts=select({"//bark/models/behavior/plan:_planner_uct" : ["-D PLANNER_UCT"], "//conditions:default": []})
  + select({"//bark/world/evaluation/ltl:_ltl_rules" : ["-D LTL_RULES"], "//conditions:default": []}),
  linkshared = 1,
  linkstatic = 1
)

# needed for external python wrapping (planners)
cc_library(
    name = "common",
    hdrs = ["common.hpp", "polymorphic_conversion.hpp"],
    deps = [
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen",
    ]
)

# needed to test external python wrapping of planners in their own bark
cc_library(
    name = "behavior",
    hdrs = ["models/behavior.hpp"],
    srcs = ["models/behavior.cpp", "polymorphic_conversion.cpp"],
    deps = [
    "//bark/python_wrapper/models/plan:planners",
    ":common",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen",
    "//bark/models/behavior/constant_acceleration:constant_acceleration",
    "//bark/models/behavior/motion_primitives:motion_primitives",
    "//bark/models/behavior/dynamic_model:dynamic_model",
    "//bark/models/behavior/idm:idm_classic",
    "//bark/models/behavior/idm:idm_lane_tracking",
    "//bark/models/behavior/static_trajectory",
    "//bark/models/behavior/idm/stochastic:stochastic",
    ]
)
