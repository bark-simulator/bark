cc_library(
    name = "_rss_interface",
    srcs = select({
        "//bark/world/evaluation/rss:_rss": ["rss_interface.cpp"],
        "//conditions:default": [],
    }),
    hdrs = select({
        "//bark/world/evaluation/rss:_rss": ["rss_interface.hpp"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
    deps = [
        "//bark/commons",
        "//bark/geometry",
        "//bark/models/dynamic",
        "//bark/world:world",
        "@boost//:geometry",
        "@boost//:filesystem",
    ] + select({"//bark/world/evaluation/rss:_rss" : ["@ad_rss_lib//:ad_rss", "@ad_rss_lib//:ad_rss_map_integration", "@map_support//:ad_physics","@spdlog" ], "//conditions:default": []}),
    copts = select({
        "//bark/world/evaluation/rss:_rss": ["-D RSS"],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "evaluator_rss",
    hdrs = ["evaluator_rss.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/world:world",
        "//bark/world/evaluation:base_evaluator",
        ":_rss_interface",
    ]
)

config_setting(
    name = "_rss",
    define_values = {"rss": "true"},
)