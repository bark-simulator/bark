package(default_visibility = ["//visibility:public"])

cc_library(
    name = "distribution",
    hdrs = [
        "distribution.hpp",
        "distributions_1d.hpp",
        "multivariate_normal.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_eigen_eigen//:eigen",
        "//bark/commons:base_type",
        "@boost//:random",
        "@boost//:math",
        "@boost//:multiprecision"
    ]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
