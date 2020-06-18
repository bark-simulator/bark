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
        "//modules/world/opendrive:opendrive",
        "//modules/world/map:road_corridor",
        "//modules/world/map:map_interface",
        "//modules/world/tests:make_test_xodr_map",
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
        "//modules/world/map:roadgraph",
        "//modules/world/opendrive",
        "//modules/world/tests:make_test_xodr_map",
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
        "//modules/world/map:map_interface",
        "//modules/world/opendrive",
        "//modules/world/tests:make_test_xodr_map",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_opendrive_tests",
  srcs = ["py_opendrive_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../../python/'],
  deps = [],
)

py_test(
  name = "py_map_interface_tests",
  srcs = ["py_map_interface_tests.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data'],
  imports = ['../../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime"],
)

py_test(
  name = "py_roadgraph_test",
  srcs = ["py_roadgraph_test.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data'],
  imports = ['../../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",],
)

py_test(
  name = "py_road_corridor_tests",
  srcs = ["py_road_corridor_tests.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data'],
  imports = ['../../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime"]
)