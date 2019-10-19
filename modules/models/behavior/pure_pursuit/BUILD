cc_library(
    name = "pure_pursuit",
    srcs = [
        "pure_pursuit.cpp",
    ],
    hdrs = [
        "pure_pursuit.hpp",
    ],
    deps = [
        "//modules/geometry:geometry",
        "//modules/models/behavior:behavior",
        "//modules/world:world"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
