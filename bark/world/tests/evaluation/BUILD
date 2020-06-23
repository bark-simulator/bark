cc_library(
    name = "constant_label_evaluator",
    hdrs = ["constant_label_evaluator.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world:include",
        "//bark/world/evaluation:base_evaluator",
    ] + select({
        "//bark/world/evaluation/ltl:_ltl_rules": [
            "@rule_monitor_project//ltl:rule_monitor",
        ],
        "//conditions:default": [],
    }),
)

cc_test(
    name = "label_test",
    srcs = ["label_test.cc"],
    copts = ["-Iexternal/gtest/include"] + select({
        "//bark/world/evaluation/ltl:_ltl_rules": ["-D LTL_RULES"],
        "//conditions:default": [],
    }),
    deps = [
        "@gtest//:gtest_main",
        "//bark/world/tests:make_test_world",
        "//bark/models/behavior/motion_primitives:motion_primitives",
    ] + select({
        "//bark/world/evaluation/ltl:_ltl_rules": [
            "//bark/world/evaluation/ltl/label_functions:label_function",
        ],
        "//conditions:default": [],
    }),
)

py_test(
    name = "py_evaluator_ltl_tests",
    srcs = select({
        "//bark/world/evaluation/ltl:_ltl_rules": ["py_evaluator_ltl_tests.py"],
        "//conditions:default": ["py_dummy_test.py"],
    }),
    data = [
        "//bark/python_wrapper:core.so",
        "//bark/runtime/tests:xodr_data",
    ],
    imports = ["../../../python/"],
    main = select({
        "//bark/world/evaluation/ltl:_ltl_rules": "py_evaluator_ltl_tests.py",
        "//conditions:default": "py_dummy_test.py",
    }),
    deps = [
        "//bark/runtime/commons:parameters",
        "//bark/runtime/scenario/scenario_generation",
    ],
)
