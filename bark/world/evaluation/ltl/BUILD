cc_library(
    name = "evaluator_ltl",
    srcs = glob(["*.cpp"]),
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/evaluation/ltl/labels:label_evaluation",
        "@rule_monitor_project//ltl:rule_monitor",
    ],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
