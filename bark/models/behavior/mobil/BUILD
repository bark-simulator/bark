cc_library(
    name = "mobil",
    srcs = [
        "mobil.cpp"
    ],
    hdrs = [
        "mobil.hpp"
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/idm:idm_classic",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet",
    ],
    visibility = ["//visibility:public"]
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)