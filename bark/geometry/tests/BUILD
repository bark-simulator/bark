cc_test(
    name = "geometry_test",
    srcs = [
        "geometry_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry:geometry",
        "//bark/world:world",
        "@gtest//:gtest_main",
    ],
)

py_test(
  name = "py_geometry_tests",
  srcs = ["py_geometry_tests.py"],
  data = ['//bark:generate_core'],
  deps = [],
  visibility = ["//visibility:public"],
)