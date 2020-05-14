cc_library(
    name = "longitudinal_acceleration",
    srcs = [
        "longitudinal_acceleration.cpp",
    ],
    hdrs = [
        "longitudinal_acceleration.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior:behavior",
        "//bark/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
