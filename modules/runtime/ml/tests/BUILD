
py_test(
    name = "py_runtime_rl_tests",
    srcs = ["py_runtime_rl_tests.py"],
    data = ['//python:bark.so',
            '//modules/runtime/tests:xodr_data'],
    imports = ['../../../python'],
    deps = ["//modules/runtime/ml:ml",
            "//modules/runtime/commons:parameters",
            "//modules/runtime/commons:commons",
            "//modules/runtime/scenario/scenario_generation:scenario_generation",
            "//modules/runtime/commons:xodr_parser"],
    visibility = ["//visibility:public"],
)


py_test(
    name = "py_runtime_tfa_tests",
    srcs = ["py_runtime_tfa_tests.py"],
    data = ['//python:bark.so',
            '//modules/runtime/tests:xodr_data'],
    imports = ['../../../python'],
    deps = ["//modules/runtime/ml:ml",
            "//modules/runtime/commons:parameters",
            "//modules/runtime/commons:commons",
            "//modules/runtime/scenario/scenario_generation:scenario_generation",
            "//modules/runtime/commons:xodr_parser"],
    visibility = ["//visibility:public"],
)


py_test(
    name = "py_runtime_tfa_sac_tests",
    srcs = ["py_runtime_tfa_sac_tests.py"],
    data = ['//python:bark.so',
            '//modules/runtime/tests:xodr_data'],
    imports = ['../../../python'],
    deps = ["//modules/runtime/ml:ml",
            "//modules/runtime/commons:parameters",
            "//modules/runtime/commons:commons",
            "//modules/runtime/scenario/scenario_generation:scenario_generation",
            "//modules/runtime/commons:xodr_parser"],
    visibility = ["//visibility:public"],
)

