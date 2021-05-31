cc_library(
    name = "behavior_simplex_sampling",
    srcs = [
        "behavior_simplex_sampling.cpp",
    ],
    hdrs = [
        "behavior_simplex_sampling.hpp",
    ],
    copts = ["-Iexternal/gtest/include"] + select({
        "//bark/world/evaluation/rss:_rss": ["-D RSS"],
        "//conditions:default": [],
    }),
    deps = [
        "//bark/models/behavior/behavior_rss:behavior_rss"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)