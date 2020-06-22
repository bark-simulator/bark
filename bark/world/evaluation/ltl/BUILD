config_setting(
    name = "_ltl_rules",
    define_values = {
        "ltl_rules": "true",
    },
)

cc_library(
    name = "evaluator_ltl",
    srcs = select({
        ":_ltl_rules": glob(["*.cpp"]),
        "//conditions:default": ["evaluator_ltl.cpp"],
    }),
    hdrs = select({
        ":_ltl_rules": glob(["*.hpp"]),
        "//conditions:default": ["evaluator_ltl.hpp"],
    }),
    copts = select({
        ":_ltl_rules": ["-D LTL_RULES"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
    ] + select({
        ":_ltl_rules": [
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
