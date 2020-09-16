cc_test(
    name = "label_test",
    srcs = ["label_test.cc"],
    copts = ["-Iexternal/gtest/include"],
    deps = [
        "//bark/models/behavior/motion_primitives",
        "//bark/world/evaluation/ltl/label_functions:label_function",
        "//bark/world/tests:make_test_world",
        "@gtest//:gtest_main",
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

py_test(
    name = "py_evaluator_rss_tests",
    srcs = select({
        "//bark/world/evaluation/rss:_rss": ["py_evaluator_rss_tests.py"],
        "//conditions:default": ["py_dummy_test.py"],
    }),
    data = [
        "//bark/python_wrapper:core.so",
        "//bark/runtime/tests:xodr_data",
    ],
    imports = ["../../../python/"],
    main = select({
        "//bark/world/evaluation/rss:_rss": "py_evaluator_rss_tests.py",
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
        "//bark/world/evaluation",
        "//bark/world/tests:make_test_world",
        "@gtest//:gtest_main",
    ],
)
