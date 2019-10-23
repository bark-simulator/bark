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
        "//modules/models/tests:make_test_world",
        "//modules/world/evaluation:evaluation",
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
        "//modules/world/map:roadgraph",
        "//modules/world/map:map_interface",
        "//modules/world/opendrive",
        "//modules/models/tests:make_test_world",
        "@gtest//:main",
    ],
)

cc_test(
    name = "local_map_test",
    srcs = [
        "local_map_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/world/map:frenet",
        "//modules/world/map:roadgraph",
        "//modules/world/map:local_map",
        "//modules/world/map:map_interface",
        "//modules/world/opendrive",
        "//modules/world:world",
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
  deps = ["//modules/runtime/commons:parameters"],
)

py_test(
  name = "py_roadgraph_test",
  srcs = ["py_roadgraph_test.py"],
  data = ['//python:bark.so'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters"],
)