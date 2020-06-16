filegroup(
    name = "params2",
    srcs = glob(["params/**"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "make_test_xodr_map",
    srcs = [
        "make_test_xodr_map.cpp",
    ],
    hdrs = [
        "make_test_xodr_map.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
    ],
)

cc_library(
    name = "make_test_world",
    srcs = [
        "make_test_world.cpp",
    ],
    hdrs = [
        "make_test_world.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/execution/interpolation",
        "//modules/world/tests:make_test_xodr_map",
        "@com_github_eigen_eigen//:eigen",
    ],
)

cc_library(
    name = "dummy_road_corridor",
    srcs = [],
    hdrs = [
        "dummy_road_corridor.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/geometry",
        "//modules/world/map:map_interface",
    ],
)

cc_test(
    name = "opendrive_tests",
    srcs = [
        "opendrive_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/world",
        "//modules/world/opendrive",
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
        ":make_test_xodr_map",
        "//modules/world/map:map_interface",
        "//modules/world/map:road_corridor",
        "//modules/world/opendrive",
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
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/execution/interpolation",
        "//modules/world",
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
        ":dummy_road_corridor",
        ":make_test_world",
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/execution/interpolation",
        "//modules/world",
        "//modules/world/evaluation",
        "//modules/world/map:map_interface",
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
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
        ":dummy_road_corridor",
        ":make_test_world",
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/execution/interpolation",
        "//modules/world",
        "//modules/world/evaluation",
        "//modules/world/map:map_interface",
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
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
        ":make_test_xodr_map",
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
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
        ":make_test_xodr_map",
        "//modules/world/map:map_interface",
        "//modules/world/opendrive",
        "@gtest//:gtest_main",
    ],
)

py_test(
    name = "py_opendrive_tests",
    srcs = ["py_opendrive_tests.py"],
    data = ["//python:bark.so"],
    imports = ["../../../python/"],
    deps = [],
)

py_test(
    name = "py_map_interface_tests",
    srcs = ["py_map_interface_tests.py"],
    data = [
        "//modules/runtime/tests:xodr_data",
        "//python:bark.so",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
        "//modules/runtime/commons:xodr_parser",
    ],
)

py_test(
    name = "py_agent_tests",
    srcs = ["py_agent_tests.py"],
    data = ["//python:bark.so"],
    imports = ["../../../python/"],
    deps = ["//modules/runtime/commons:parameters"],
)

py_test(
    name = "py_world_tests",
    srcs = ["py_world_tests.py"],
    data = ["//python:bark.so"],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
    ],
)

py_test(
    name = "py_system_tests",
    srcs = ["py_system_tests.py"],
    data = [
        ":params2",
        "//python:bark.so",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
        "//modules/runtime/viewer:video_renderer",
    ],
)

py_test(
    name = "py_roadgraph_test",
    srcs = ["py_roadgraph_test.py"],
    data = [
        "//modules/runtime/tests:xodr_data",
        "//python:bark.so",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime/commons:parameters",
        "//modules/runtime/commons:xodr_parser",
    ],
)

py_test(
    name = "py_road_corridor_tests",
    srcs = ["py_road_corridor_tests.py"],
    data = [
        "//modules/runtime/tests:xodr_data",
        "//python:bark.so",
    ],
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime",
        "//modules/runtime/commons:parameters",
        "//modules/runtime/commons:xodr_parser",
    ],
)

cc_test(
    name = "rss_interface_test",
    srcs = ["rss_interface_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    data = ["//modules/runtime/tests:xodr_data"],
    deps = [
        "//modules/commons/params",
        "//modules/geometry",
        "//modules/models/behavior/constant_velocity",
        "//modules/models/execution/interpolation",
        "//modules/world",
        "//modules/world/evaluation:rss_interface",
        "//modules/world/map:map_interface",
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
        "@gtest//:gtest_main",
    ],
)
