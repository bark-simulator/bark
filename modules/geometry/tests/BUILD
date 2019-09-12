cc_test(
    name = "geometry_test",
    srcs = [
        "geometry_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry:geometry",
        "//modules/world:world",
        "@gtest//:main",
        "@com_github_google_glog//:glog",
    ],
)

py_test(
  name = "py_geometry_tests",
  srcs = ["py_geometry_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../python/'],
  deps = [],
)
