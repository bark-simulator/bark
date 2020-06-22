cc_library(
    name = "base_label_evaluator",
    hdrs = ["base_label_function.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world:include",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_library(
    name = "label_evaluation",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        ":base_label_evaluator",
        "//bark/world:include",
        "//bark/world/map:road_corridor",
        "//bark/world/evaluation:base_evaluator",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = ["@rule_monitor_project//ltl:rule_monitor"],
)
