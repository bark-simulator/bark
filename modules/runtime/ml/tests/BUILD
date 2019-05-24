
py_test(
    name = "py_runtime_rl_tests",
    srcs = ["py_runtime_rl_tests.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python',
               '//modules/runtime/tests:xodr_data'],
    deps = ["//modules/runtime/ml:ml",
            "//modules/runtime/commons:parameters",
            "//modules/runtime/commons:commons",
            "//modules/runtime/scenario/scenario_generation:scenario_generation",
            "//modules/runtime/commons:xodr_parser",
            "//modules/runtime/commons:roadgraph_generator"],
    visibility = ["//visibility:public"],
)