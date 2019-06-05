cc_test(
    name = "dynamic_tests",
    srcs = [
        "dynamic_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic:dynamic",
        "@gtest//:main",
        "@boost//:geometry",
    ],
)

cc_test(
    name = "execution_test",
    srcs = [
        "execution_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic:dynamic",
        "//modules/models/execution/mpc:mpc",
        "//modules/models/execution/interpolation:interpolation",
        "@gtest//:main",
    ],
)

cc_test(
    name = "behavior_test",
    srcs = [
        "behavior_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic:dynamic",
        "//modules/models/behavior/motion_primitives:motion_primitives",
        "@gtest//:main",
    ],
)