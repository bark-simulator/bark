cc_library(
    name = "constant_acceleration",
    srcs = [
        "constant_acceleration.cpp",
    ],
    hdrs = [
        "constant_acceleration.hpp",
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