cc_library(
    name = "frenet",
    srcs = [
        "frenet.cpp",
    ],
    hdrs = [
        "frenet.hpp",
    ],
    deps = [
        "//modules/geometry",
        #"//modules/world/opendrive",
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
