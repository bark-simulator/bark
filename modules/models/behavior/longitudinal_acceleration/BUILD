cc_library(
    name = "longitudinal_acceleration",
    srcs = [
        "longitudinal_acceleration.cpp",
    ],
    hdrs = [
        "longitudinal_acceleration.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior:behavior",
        "//modules/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
