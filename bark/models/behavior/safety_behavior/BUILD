cc_library(
    name = "safety_behavior",
    srcs = [
        "safety_behavior.cpp",
    ],
    hdrs = [
        "safety_behavior.hpp",
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