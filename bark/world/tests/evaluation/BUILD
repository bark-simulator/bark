cc_library(
    name = "constant_label_evaluator",
    hdrs = ["constant_label_evaluator.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world:include",
        "//bark/world/evaluation:base_evaluator",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_test(
    name = "label_test",
    srcs = ["label_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/world/tests:make_test_world",
        "//bark/world/evaluation/ltl/labels:label_evaluation",
        "//bark/models/behavior/motion_primitives:motion_primitives",
        "@gtest//:gtest_main",
    ],
)

py_test(
    name = "py_evaluator_ltl_tests",
    srcs = ["py_evaluator_ltl_tests.py"],
        data = [
            "//bark/runtime/tests:xodr_data",
            "//bark/python_wrapper:core.so",
        ],
        imports = ["../../../python/"],
        deps = [
            "//bark/runtime/commons:parameters",
            "//bark/runtime/scenario/scenario_generation",
        ],
)