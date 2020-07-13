package(default_visibility = ["//visibility:public"])

cc_test(
    name = "params_tests",
    srcs = [
        "params_tests.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/commons:commons",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "util_tests",
    srcs = [
        "util_tests.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/commons:commons",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "distribution_tests",
    srcs = [
        "distribution_tests.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/commons:commons",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "frenet_test",
    srcs = [
        "frenet_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/commons/transformation:frenet",
        "@gtest//:gtest_main",
    ],
)



py_test(
  name = "py_commons_tests",
  srcs = ["py_commons_tests.py"],
  data = ['//bark:generate_core'],
  deps = ["//bark/runtime/commons:commons"],
)
