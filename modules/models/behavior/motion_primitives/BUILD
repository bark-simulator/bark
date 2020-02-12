cc_library(
    name = "motion_primitives",
    srcs = [
        "motion_primitives.cpp",
        "continuous_actions.cpp",
    ],
    hdrs = [
        "motion_primitives.hpp",
        "continuous_actions.hpp",
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
