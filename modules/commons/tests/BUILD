cc_test(
    name = "params_tests",
    srcs = [
        "params_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/commons:commons",
        "@gtest//:main",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "util_tests",
    srcs = [
        "util_tests.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/commons:commons",
        "@gtest//:main",
        "@com_github_google_glog//:glog",
    ],
)



py_test(
  name = "py_commons_tests",
  srcs = ["py_commons_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../python/'],
  deps = ["//modules/runtime/commons:parameters"],
)
