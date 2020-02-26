package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "bark.so",
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
    "//modules/runtime/tests:py_param_server_test_helper",
    "//modules/geometry:geometry",
    "//modules/world/opendrive:opendrive",
    "//modules/world/tests:make_test_xodr_map",
    "//modules/world/tests:make_test_world",
    "//modules/world/map:roadgraph",
    "//modules/world/goal_definition:goal_definition",
    "//modules/world/evaluation:evaluation",
    "//modules/commons/params:params",
    "//modules/commons/util:util",
    "//modules/models/execution/interpolation:interpolation",
    "//modules/models/behavior/constant_velocity:constant_velocity",
    "//modules/models/behavior/motion_primitives:motion_primitives",
    "//modules/models/behavior/dynamic_model:dynamic_model",
    "//modules/models/behavior/idm:idm_classic",
    "//modules/models/behavior/idm:idm_lane_tracking",
    "//modules/models/behavior/mobil:mobil",
    "//modules/models/behavior/static_trajectory",
    #"//modules/models/execution/mpc:mpc",
    "//modules/runtime/viewer:viewer",
    "//modules/world:world",
    "//modules/runtime:cc_runtime",
    "//python/models/plan:planners"
  ] + select({"//modules/models/behavior/plan:_planner_uct" : ["@planner_uct//python:planner_uct"], "//conditions:default": []}),
  copts=select({"//modules/models/behavior/plan:_planner_uct" : ["-D PLANNER_UCT"], "//conditions:default": []}), 
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

# needed to test external python wrapping of planners in their own modules
cc_library(
    name = "behavior",
    hdrs = ["models/behavior.hpp"],
    srcs = ["models/behavior.cpp", "polymorphic_conversion.cpp"],
    deps = [
    "//python/models/plan:planners",
    ":common",
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
    "//modules/models/behavior/constant_velocity:constant_velocity",
    "//modules/models/behavior/motion_primitives:motion_primitives",
    "//modules/models/behavior/dynamic_model:dynamic_model",
    "//modules/models/behavior/idm:idm_classic",
    "//modules/models/behavior/idm:idm_lane_tracking",
    "//modules/models/behavior/static_trajectory",
    ]
)
