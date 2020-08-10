package(default_visibility = ["//visibility:public"])

cc_test(
    name = "dynamic_tests",
    srcs = [
        "dynamic_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet",
        "@gtest//:gtest_main",
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
        "//bark/geometry",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet",
        #"//bark/models/execution/mpc:mpc",
        "//bark/models/execution/interpolation:interpolation",
        "@gtest//:gtest_main",
    ],
)

cc_test(
    name = "behavior_motion_primitive_test",
    srcs = [
        "behavior_motion_primitive_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/models/dynamic:dynamic",
        "//bark/models/behavior/motion_primitives:motion_primitives",
        "//bark/models/behavior/idm:idm_lane_tracking",
        "//bark/world:world",
        "//bark/world/tests:make_test_world",
        "@gtest//:gtest_main",
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
        "//bark/geometry",
        "//bark/models/behavior/idm:idm_classic",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/execution/interpolation:interpolation",
        "@gtest//:gtest_main",
        "@com_github_eigen_eigen//:eigen",
        "//bark/world/tests:make_test_world",
    ],
)


cc_test(
    name = "behavior_mobil_test",
    srcs = [
        "behavior_mobil_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [        
        "//bark/geometry",
        "//bark/models/behavior/rule_based:mobil",
        "//bark/models/behavior/rule_based:mobil_behavior",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/world/tests:make_test_xodr_map",
        "@gtest//:gtest_main",
        "@com_github_gflags_gflags//:gflags",
    ],
)

py_test(
  name = "py_behavior_model_test",
  srcs = ["py_behavior_model_test.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],

  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/scenario_generation:scenario_generation"],
)

cc_test(
    name = "behavior_static_trajectory_test",
    srcs = [
        "behavior_static_trajectory_test.cc",
    ],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/geometry",
        "//bark/models/behavior/static_trajectory",
        "//bark/world/tests:make_test_world",
        "@com_github_eigen_eigen//:eigen",
        "@gtest//:gtest_main",
    ],
)
