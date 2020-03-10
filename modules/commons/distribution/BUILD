package(default_visibility = ["//visibility:public"])

cc_library(
    name = "distribution",
    hdrs = [
        "distribution.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons/params:params",
        "@com_github_eigen_eigen//:eigen"
    ]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
