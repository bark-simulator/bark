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
        "@com_github_google_glog//:glog",
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
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "behavior_motion_primitive_test",
    srcs = [
        "behavior_motion_primitive_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic:dynamic",
        "//modules/models/behavior/motion_primitives:motion_primitives",
        "//modules/world:world",
        "@gtest//:main",
        "@com_github_google_glog//:glog",
        "@com_github_eigen_eigen//:eigen",
    ],
)

cc_test(
    name = "behavior_idm_classic_test",
    srcs = [
        "behavior_idm_classic_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/execution/interpolation:interpolation",
        "@gtest//:main",
        "@com_github_google_glog//:glog",
        "@com_github_eigen_eigen//:eigen",
        ":make_test_world",
    ],
)

cc_library(
    name="make_test_world",
    srcs=[
        "make_test_world.cpp"
    ],
    hdrs = [
        "make_test_world.hpp"
    ],
    deps = [
        "//modules/geometry",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/execution/interpolation:interpolation",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)
