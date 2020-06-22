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
        "//bark/world/tests:make_test_xodr_map",
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
        "//bark/world/tests:make_test_xodr_map",
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
        "//bark/world/tests:make_test_xodr_map",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_opendrive_tests",
  srcs = ["py_opendrive_tests.py"],
  data = ['//bark:generate_core'],
  imports = ['../../../../python/'],
  deps = [],
  visibility = ["//visibility:public"],
)

py_test(
  name = "py_map_interface_tests",
  srcs = ["py_map_interface_tests.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  imports = ['../../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime:runtime"],
  visibility = ["//visibility:public"],
)

py_test(
  name = "py_roadgraph_test",
  srcs = ["py_roadgraph_test.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  imports = ['../../../../python/'],
  deps = ["//bark/runtime/commons:commons"],
  visibility = ["//visibility:public"],
)

py_test(
  name = "py_road_corridor_tests",
  srcs = ["py_road_corridor_tests.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  imports = ['../../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime:runtime"],
  visibility = ["//visibility:public"],
)