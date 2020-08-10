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
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
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
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
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
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/world/evaluation:evaluation",
        "//bark/models/behavior/motion_primitives:motion_primitives",
        ":make_test_world",
        ":dummy_road_corridor",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_world_tests",
  srcs = ["py_world_tests.py"],
  data = ['//bark:generate_core'],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime"],
  visibility = ["//visibility:public"],
)

py_test(
  name = "py_system_tests",
  srcs = ["py_system_tests.py"],
  data = ['//bark:generate_core',
          ':params2'],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime",
          "//bark/runtime/viewer:video_renderer"],
  visibility = ["//visibility:public"],
)