package(default_visibility = ["//visibility:public"])

cc_library(
    name = "distribution",
    hdrs = [
        "distribution.hpp",
        "distributions_1d.hpp",
        "multivariate_distribution.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "//modules/commons:base_type",
        "@boost//:math"
    ]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
