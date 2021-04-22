cc_library(
    name = "safe_distances",
    srcs = [
        "evaluator_dynamic_safe_dist_long.cpp" ,
        "evaluator_static_safe_dist.cpp",
        "evaluator_safe_dist_drivable_area.cpp"
    ],
    hdrs = [
            "evaluator_safe_dist_base.hpp", 
            "evaluator_dynamic_safe_dist_long.hpp",
            "evaluator_static_safe_dist.hpp",
            "evaluator_safe_dist_drivable_area.hpp"
    ],
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
