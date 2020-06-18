cc_library(
    name = "base_label_evaluator",
    hdrs = ["base_label_function.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/world:include",
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
        "//modules/world:include",
        "//modules/world/map:road_corridor",
        "//modules/world/evaluation:base_evaluator",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = ["@rule_monitor_project//ltl:rule_monitor"],
)
