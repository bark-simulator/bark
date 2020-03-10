cc_library(
    name = "stochastic",
    srcs = [
        "idm_stochastic_headway.cpp",
    ],
    hdrs = [
        "behavior_stochastic.hpp",
        "idm_stochastic_headway.hpp",
    ],
    deps = [
        "//modules/models/behavior:behavior"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)