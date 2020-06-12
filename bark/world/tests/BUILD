package(default_visibility = ["//visibility:public"])

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
        "//bark/world/map:roadgraph",
        "//bark/world/opendrive",
        "//bark/geometry:geometry",
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
        "//bark/geometry",
        "//bark/models/behavior/idm:idm_classic",
        "//bark/models/behavior/constant_velocity:constant_velocity",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/world/tests:make_test_xodr_map",
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
        "//bark/world/map:map_interface",
        "//bark/geometry",
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
        "//bark/geometry",
        "//bark/world/opendrive",
        "//bark/world:world",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "road_corridor_tests",
    srcs = [
        "road_corridor_tests.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/world/opendrive:opendrive",
        "//bark/world/map:road_corridor",
        "//bark/world/map:map_interface",
        ":make_test_xodr_map",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "agent_test",
    srcs = [
        "agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/world:world",
        "//bark/models/behavior/constant_velocity:constant_velocity",
        "//bark/models/execution/interpolation:interpolation",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "world_test",
    srcs = [
        "world_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/world:world",
        "//bark/world/map:map_interface",
        "//bark/world/map:roadgraph",
        "//bark/world/opendrive:opendrive",
        "//bark/models/behavior/constant_velocity:constant_velocity",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/world/evaluation:evaluation", 
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
        "//bark/geometry",
        "//bark/world:world",
        "//bark/world/map:map_interface",
        "//bark/world/map:roadgraph",
        "//bark/world/opendrive:opendrive",
        "//bark/models/behavior/constant_velocity:constant_velocity",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/world/evaluation:evaluation",
        "//bark/models/behavior/motion_primitives:motion_primitives",
        ":make_test_world",
        ":dummy_road_corridor",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "roadgraph_test",
    srcs = [
        "roadgraph_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/world/map:roadgraph",
        "//bark/world/opendrive",
        ":make_test_xodr_map",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "map_interface_test",
    srcs = [
        "map_interface_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/world/map:map_interface",
        "//bark/world/opendrive",
        ":make_test_xodr_map",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_opendrive_tests",
  srcs = ["py_opendrive_tests.py"],
  data = ['//bark:generate_core'],
  deps = [],
)

py_test(
  name = "py_map_interface_tests",
  srcs = ["py_map_interface_tests.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime:runtime"],
)

py_test(
  name = "py_agent_tests",
  srcs = ["py_agent_tests.py"],
  data = ['//bark:generate_core'],
  deps = ["//bark/runtime/commons:commons"],
)

py_test(
  name = "py_world_tests",
  srcs = ["py_world_tests.py"],
  data = ['//bark:generate_core'],
  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime:runtime"],
)

py_test(
  name = "py_system_tests",
  srcs = ["py_system_tests.py"],
  data = ['//bark:generate_core',
          ':params2'],
  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime:runtime",
          "//bark/runtime/viewer:video_renderer"],
)

py_test(
  name = "py_roadgraph_test",
  srcs = ["py_roadgraph_test.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  deps = ["//bark/runtime/commons:commons",],
)

py_test(
  name = "py_road_corridor_tests",
  srcs = ["py_road_corridor_tests.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime:runtime"]
)