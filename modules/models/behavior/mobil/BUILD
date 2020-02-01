cc_library(
    name = "mobil",
    srcs = [
        "mobil.cpp"
    ],
    hdrs = [
        "mobil.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/idm:idm_classic",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet",
    ],
    visibility = ["//visibility:public"]
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)