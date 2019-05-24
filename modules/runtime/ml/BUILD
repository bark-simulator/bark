py_library(
    name = "ml",
    srcs = ["action_wrapper.py",
            "nn_state_observer.py",
            "reward_observer.py",
            "runtime_rl.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = ["//modules/runtime:runtime"],
    visibility = ["//visibility:public"],
)