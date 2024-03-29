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
    copts = ["-Iexternal/gtest/include"] + select({
        "//bark/world/evaluation/rss:_rss": ["-D RSS"],
        "//conditions:default": [],
    }),
    data = ["//bark/runtime/tests:xodr_data",
            "//bark/world/tests/map:csv_map_data"],
    deps = [
        "//bark/geometry",
        "//bark/world:world",
        "//bark/world/map:map_interface",
        "//bark/world/map:roadgraph",
        "//bark/world/opendrive:opendrive",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/world/evaluation:evaluation",
        "//bark/world/evaluation/safe_distances:safe_distances",
        ":make_test_world",
        ":dummy_road_corridor",
        "@gtest//:gtest_main",
        ] + select({
        "//bark/world/evaluation/rss:_rss": ["//bark/world/evaluation/rss:evaluator_rss"],
        "//conditions:default": [],
    }),
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
  data = ["//bark:generate_core",
          "//bark/world/tests/map:csv_map_data"],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime"],
  visibility = ["//visibility:public"],
)

py_library(
    name = "python_behavior_model",
    srcs = ["python_behavior_model.py"],
    data = ["//bark:generate_core"],
    imports = ['../../../python'],
    deps = ["//bark/runtime/commons:parameters"],
    visibility = ["//visibility:public"],
)

py_test(
  name = "py_system_tests",
  srcs = ["py_system_tests.py"],
  data = ["//bark:generate_core",
          ':params2'],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime",
          "//bark/world/tests:python_behavior_model",
          "//bark/runtime/viewer:video_renderer"],
  visibility = ["//visibility:public"],
)