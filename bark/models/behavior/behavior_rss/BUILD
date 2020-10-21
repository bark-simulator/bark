cc_library(
    name = "behavior_rss",
    srcs = [
        "behavior_rss.cpp",
    ],
    hdrs = [
        "behavior_rss.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/behavior_safety:behavior_safety",
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