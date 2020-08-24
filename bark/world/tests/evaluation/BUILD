
cc_test(
    name = "label_test",
    srcs = ["label_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:gtest_main",
        "//bark/world/tests:make_test_world",
        "//bark/models/behavior/motion_primitives:motion_primitives",
        "//bark/world/evaluation/ltl/label_functions:label_function",
    ],
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

cc_test(
    name = "commons_test",
    srcs = ["commons_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "@gtest//:gtest_main",
        "//bark/world/tests:make_test_world",
        "//bark/world/evaluation:evaluation",
    ],
)
