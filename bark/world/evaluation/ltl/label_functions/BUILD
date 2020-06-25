cc_library(
    name = "base_label_function",
    hdrs = ["base_label_function.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world:include",
        "//bark/world/evaluation/ltl/label:include",
    ],
)

cc_library(
    name = "label_function",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        ":base_label_function",
        "//bark/world:include",
        "//bark/world/map:road_corridor",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/evaluation/ltl/label:include",
    ],
)

