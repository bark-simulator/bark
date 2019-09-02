cc_library(
    name = "motion_primitives",
    srcs = [
        "motion_primitives.cpp",
    ],
    hdrs = [
        "motion_primitives.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:include",
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
