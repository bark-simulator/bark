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
    copts = ["-Iexternal/gtest/include"] + select({
        "//bark/world/evaluation/ltl:_ltl": ["-D LTL"],
        "//conditions:default": [],
    }),
    deps = [
        "@gtest//:gtest_main",
        "//bark/world/tests:make_test_world",
        "//bark/models/behavior/motion_primitives:motion_primitives",
    ] + select({
        "//bark/world/evaluation/ltl:_ltl": [
            "//bark/world/evaluation/ltl/labels:label_evaluation",
        ],
        "//conditions:default": [],
    }),
)

py_test(
    name = "py_evaluator_ltl_tests",
    srcs = select({
        "//bark/world/evaluation/ltl:_ltl": ["py_evaluator_ltl_tests.py"],
        "//conditions:default": ["py_dummy_test.py"],
    }),
    data = [
        "//bark/python_wrapper:core.so",
        "//bark/runtime/tests:xodr_data",
    ],
    imports = ["../../../python/"],
    main = select({
        "//bark/world/evaluation/ltl:_ltl": "py_evaluator_ltl_tests.py",
        "//conditions:default": "py_dummy_test.py",
    }),
    deps = [
        "//bark/runtime/commons:parameters",
        "//bark/runtime/scenario/scenario_generation",
    ],
)
