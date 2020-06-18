cc_library(
    name = "evaluator_ltl",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
