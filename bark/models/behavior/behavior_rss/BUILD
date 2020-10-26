cc_library(
    name = "behavior_rss",
    srcs = [
        "behavior_rss.cpp",
    ],
    hdrs = [
        "behavior_rss.hpp",
    ],
    copts = ["-Iexternal/gtest/include"] + select({
        "//bark/world/evaluation/rss:_rss": ["-D RSS"],
        "//conditions:default": [],
    }),
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/behavior_safety:behavior_safety",
        "//bark/models/dynamic:dynamic",
        "//bark/world/evaluation:evaluation"
    ] +  select({
        "//bark/world/evaluation/rss:_rss": ["//bark/world/evaluation/rss:evaluator_rss"],
        "//conditions:default": [],
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)