cc_library(
    name = "static_trajectory",
    srcs = ["behavior_static_trajectory.cpp"],
    hdrs = ["behavior_static_trajectory.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/models/behavior",
        "//modules/world",
    ],
)

cc_library(
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
