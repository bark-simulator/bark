cc_library(
    name = "base_idm",
    srcs = [
        "base_idm.cpp",
    ],
    hdrs = [
        "base_idm.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet",
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
<<<<<<< HEAD:modules/models/behavior/idm/BUILD
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet",
        ":base_idm"
=======
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/longitudinal_acceleration:longitudinal_acceleration",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet",
>>>>>>> Package Restructuring:bark/models/behavior/idm/BUILD
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