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
    name = "opendrive_tests",
    srcs = [
        "opendrive_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/world/opendrive",
        "//modules/world:world",
        "@gtest//:main",
    ],
)

cc_test(
    name = "road_corridor_tests",
    srcs = [
        "road_corridor_tests.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/world/opendrive:opendrive",
        "//modules/world/map:road_corridor",
        "//modules/world/map:map_interface",
        ":make_test_xodr_map",
        "@gtest//:main",
    ],
)

cc_test(
    name = "agent_test",
    srcs = [
        "agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/world:world",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/execution/interpolation:interpolation",
        "@gtest//:main",
    ],
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
        "@gtest//:main",
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
        ":make_test_world",
        ":dummy_road_corridor",
        "@gtest//:main",
    ],
)

cc_test(
    name = "roadgraph_test",
    srcs = [
        "roadgraph_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
        ":make_test_xodr_map",
        "@gtest//:main",
    ],
)

cc_test(
    name = "map_interface_test",
    srcs = [
        "map_interface_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/world/map:map_interface",
        "//modules/world/opendrive",
        ":make_test_xodr_map",
        "@gtest//:main",
    ],
)

py_test(
  name = "py_opendrive_tests",
  srcs = ["py_opendrive_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../python/'],
  deps = [],
)

py_test(
  name = "py_map_interface_tests",
  srcs = ["py_map_interface_tests.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime"],
)

py_test(
  name = "py_agent_tests",
  srcs = ["py_agent_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters"],
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

py_test(
  name = "py_roadgraph_test",
  srcs = ["py_roadgraph_test.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",],
)

py_test(
  name = "py_road_corridor_tests",
  srcs = ["py_road_corridor_tests.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime"]
)