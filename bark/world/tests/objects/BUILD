cc_test(
    name = "agent_test",
    srcs = [
        "agent_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/world:world",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/execution/interpolation:interpolation",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_agent_tests",
  srcs = ["py_agent_tests.py"],
  data = ['//bark:generate_core'],
  imports = ['../../../../python/'],
  deps = ["//bark/runtime/commons:commons"],
  visibility = ["//visibility:public"],
)