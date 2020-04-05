cc_library(
    name = "stochastic",
    srcs = [
        "idm_stochastic.cpp",
    ],
    hdrs = [
        "idm_stochastic.hpp",
    ],
    deps = [
        "//modules/models/behavior/idm:idm_classic",
        "//modules/commons/distribution:distribution"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)