cc_library(
    name = "_rss_interface",
    srcs = ["rss_interface.cpp"],
    hdrs = ["rss_interface.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/commons",
        "//bark/geometry",
        "//bark/models/dynamic",
        "//bark/world",
        "@ad_rss_lib//:ad_rss",
        "@ad_rss_lib//:ad_rss_map_integration",
        "@boost//:geometry",
        "@boost//:filesystem",
        "@map_support//:ad_physics",
        "@spdlog",
    ],
)

cc_library(
    name = "evaluator_rss",
    hdrs = ["evaluator_rss.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world",
        "//bark/world/evaluation:base_evaluator",
        ":_rss_interface",
    ]
)

config_setting(
    name = "_rss",
    define_values = {"rss": "true"},
)