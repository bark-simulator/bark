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
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_agent_tests",
  srcs = ["py_agent_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../../python/'],
  deps = ["//modules/runtime/commons:parameters"],
)