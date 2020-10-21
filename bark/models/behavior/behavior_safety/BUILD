cc_library(
    name = "behavior_safety",
    srcs = [
        "behavior_safety.cpp",
    ],
    hdrs = [
        "behavior_safety.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/idm:idm_lane_tracking",
        "//bark/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)