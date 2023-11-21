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
        "//bark:generate_core",
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
        "//bark:generate_core",
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

cc_test(
    name = "rss_interface_test",
    srcs = ["rss_interface_test.cc"],
    copts = ["-Iexternal/gtest/include"] + select({
        "//bark/world/evaluation/rss:_rss": ["-D RSS"],
        "//conditions:default": [],
    }),
    data = ["//bark/runtime/tests:xodr_data"],
    deps = [
        "//bark/models/behavior/motion_primitives",
        "//bark/geometry:geometry",
        "//bark/models/dynamic:dynamic",
        "//bark/models/execution/interpolation:interpolation",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/commons/params:params",
        "//bark/world:world",
        "//bark/world/map:map_interface",
        "//bark/world/map:roadgraph",
        "//bark/world/opendrive:opendrive",
        "//bark/world/goal_definition:goal_definition",
        "//bark/world/tests:make_test_world",
        "@gtest//:gtest_main",
    ] + select({
        "//bark/world/evaluation/rss:_rss": ["//bark/world/evaluation/rss:_rss_interface"],
        "//conditions:default": [],
    }),
)
