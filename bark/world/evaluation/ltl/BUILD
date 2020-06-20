config_setting(
    name = "_ltl",
    define_values = {
        "ltl": "true",
    },
)

cc_library(
    name = "evaluator_ltl",
    srcs = select({
        ":_ltl": glob(["*.cpp"]),
        "//conditions:default": ["evaluator_ltl.cpp"],
    }),
    hdrs = select({
        ":_ltl": glob(["*.hpp"]),
        "//conditions:default": ["evaluator_ltl.hpp"],
    }),
    copts = select({
        ":_ltl": ["-D LTL"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
    ] + select({
        ":_ltl": [
            "@rule_monitor_project//ltl:rule_monitor",
            "//bark/world/evaluation/ltl/labels:label_evaluation",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
