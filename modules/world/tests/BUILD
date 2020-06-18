filegroup(
    name="params2",
    srcs=glob(["params/**"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name="make_test_xodr_map",
    srcs=[
        "make_test_xodr_map.cpp"
    ],
    hdrs = [
        "make_test_xodr_map.hpp"
    ],
    deps = [
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
        "//modules/geometry:geometry",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="make_test_world",
    srcs=[
        "make_test_world.cpp"
    ],
    hdrs = [
        "make_test_world.hpp"
    ],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/execution/interpolation:interpolation",
        "//modules/world/tests:make_test_xodr_map",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="dummy_road_corridor",
    srcs=[],
    hdrs = [
        "dummy_road_corridor.hpp"
    ],
    deps = [
        "//modules/world/map:map_interface",
        "//modules/geometry",
    ],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "world_test",
    srcs = [
        "world_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/world:world",
        "//modules/world/map:map_interface",
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive:opendrive",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/execution/interpolation:interpolation",
        "//modules/world/evaluation:evaluation", 
        ":make_test_world",
        ":dummy_road_corridor",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "observed_world_test",
    srcs = [
        "observed_world_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/world:world",
        "//modules/world/map:map_interface",
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive:opendrive",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/execution/interpolation:interpolation",
        "//modules/world/evaluation:evaluation",
        "//modules/models/behavior/motion_primitives:motion_primitives",
        ":make_test_world",
        ":dummy_road_corridor",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_world_tests",
  srcs = ["py_world_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime:runtime"],
)

py_test(
  name = "py_system_tests",
  srcs = ["py_system_tests.py"],
  data = ['//python:bark.so',
          ':params2'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime:runtime",
          "//modules/runtime/viewer:video_renderer"],
)