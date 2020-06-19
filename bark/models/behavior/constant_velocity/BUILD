cc_library(
    name = "constant_velocity",
    srcs = [
        "constant_velocity.cpp",
    ],
    hdrs = [
        "constant_velocity.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/idm:idm_classic",
        "//bark/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)