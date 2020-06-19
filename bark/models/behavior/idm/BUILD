cc_library(
    name = "base_idm",
    srcs = [
        "base_idm.cpp",
    ],
    hdrs = [
        "base_idm.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "idm_classic",
    srcs = [
        "idm_classic.cpp",
    ],
    hdrs = [
        "idm_classic.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet",
        ":base_idm"
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
        "//bark/models/behavior/idm:idm_classic",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)