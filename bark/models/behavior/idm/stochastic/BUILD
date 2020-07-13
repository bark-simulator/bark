cc_library(
    name = "stochastic",
    srcs = [
        "idm_stochastic.cpp",
    ],
    hdrs = [
        "idm_stochastic.hpp",
    ],
    deps = [
        "//bark/models/behavior/idm:idm_classic",
        "//bark/commons/distribution:distribution"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)