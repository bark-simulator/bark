py_library(
    name = "ml",
    srcs = ["action_wrapper.py",
            "nn_state_observer.py",
            "state_evaluator.py",
            "runtime_rl.py",
            "tfa_wrapper.py",
            "spaces.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = ["//modules/runtime:runtime"],
    visibility = ["//visibility:public"],
)