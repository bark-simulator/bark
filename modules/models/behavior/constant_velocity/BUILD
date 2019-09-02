cc_library(
    name = "constant_velocity",
    srcs = [
        "constant_velocity.cpp",
    ],
    hdrs = [
        "constant_velocity.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/longitudinal_acceleration:longitudinal_acceleration",
        "//modules/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
