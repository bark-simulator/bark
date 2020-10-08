cc_library(
    name = "safe_distances",
    srcs = ["evaluator_safe_dist_long.cpp"],
    hdrs = ["evaluator_safe_dist_long.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/evaluation/ltl/label_functions:label_function",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
