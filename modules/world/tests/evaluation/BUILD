cc_library(
    name = "constant_label_evaluator",
    hdrs = ["constant_label_evaluator.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/world:include",
        "//modules/world/evaluation:base_evaluator",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_test(
    name = "label_test",
    srcs = ["label_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//modules/world/tests:make_test_world",
        "//modules/world/evaluation/ltl/labels:label_evaluation",
        "//modules/models/behavior/motion_primitives:motion_primitives",
        "@gtest//:gtest_main",
    ],
)

py_test(
    name = "py_evaluator_ltl_tests",
    srcs = ["py_evaluator_ltl_tests.py"],
        data = [
            "//modules/runtime/tests:xodr_data",
            "//python:bark.so",
        ],
        imports = ["../../../python/"],
        deps = [
            "//modules/runtime/commons:parameters",
            "//modules/runtime/scenario/scenario_generation",
        ],
)