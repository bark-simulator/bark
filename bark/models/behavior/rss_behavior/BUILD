cc_library(
    name = "rss_behavior",
    srcs = [
        "rss_behavior.cpp",
    ],
    hdrs = [
        "rss_behavior.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/safety_behavior:safety_behavior",
        "//bark/models/dynamic:dynamic",
        "//bark/world/evaluation/rss:evaluator_rss",
        "//bark/world/evaluation:evaluation"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)