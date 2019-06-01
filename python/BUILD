package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "bark.so",
  srcs = glob([
    "*.cpp", "*.hpp",
    "commons/*.cpp", "commons/*.hpp",
    "geometry/*.cpp", "geometry/*.hpp",
    "world/*.cpp", "world/*.hpp",
    "models/*.cpp", "models/*.hpp",
    "viewer/*.cpp", "viewer/*.hpp"]),
  deps = [
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
	  "@com_github_eigen_eigen//:eigen", 
    "//modules/geometry:geometry",
    "//modules/world/opendrive:opendrive",
    "//modules/world/map:roadgraph",
    "//modules/world/goal_definition:goal_definition",
    "//modules/world/evaluation:evaluation",
    "//modules/commons/params:params",
    "//modules/models/execution/interpolation:interpolation",
    "//modules/models/behavior/constant_velocity:constant_velocity",
    "//modules/models/behavior/state_delta:state_delta",
    "//modules/models/execution/mpc:mpc",
    "//modules/runtime/viewer:viewer",
    "//modules/world:world",
  ],
  linkshared = 1,
  linkstatic = 1
)
