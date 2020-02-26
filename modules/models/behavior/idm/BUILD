cc_library(
    name = "idm_classic",
    srcs = [
        "idm_classic.cpp",
    ],
    hdrs = [
        "idm_classic.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/longitudinal_acceleration:longitudinal_acceleration",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "idm_lane_tracking",
    srcs = [
        "idm_lane_tracking.cpp",
    ],
    hdrs = [
        "idm_lane_tracking.hpp",
    ],
    deps = [
        "//modules/models/behavior/idm:idm_classic",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)