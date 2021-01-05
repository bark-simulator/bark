# config_setting(
#     name = "_ltl_rules",
#     define_values = {
#         "ltl_rules": "true",
#     },
# )

cc_library(
    name = "evaluator_ltl",
    srcs = ["evaluator_ltl.cpp"],
    hdrs = ["evaluator_ltl.hpp"],
    copts = select({
        "false": ["-D LTL_RULES"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/evaluation/ltl/label_functions:label_function",
    ] + select({
        "false": [
            "@rule_monitor_project//ltl:rule_monitor",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    deps = [
        "//bark/world/evaluation/ltl/label_functions:include",
        "//bark/world/evaluation/ltl/label:include"
    ],
    visibility = ["//visibility:public"],
)
