cc_library(
    name = "frenet",
    srcs = [
        "frenet.cpp",
        "frenet_state.cpp",
    ],
    hdrs = [
        "frenet.hpp",
        "frenet_state.hpp",
    ],
    deps = [
        "//modules/geometry",
        "//modules/models/dynamic",
        "@boost//:geometry",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
