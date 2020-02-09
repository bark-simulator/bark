package(default_visibility = ["//visibility:public"])

cc_library(
    name = "params",
    hdrs = [
        "params.hpp",
        "params_test.h",
        "default_params.hpp",
        "setter_params.hpp"
    ],
    srcs = [
        "setter_params.cpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons/util:util",
        "@boost//:geometry", # add since sometimes "boost/variant.hpp" not found
    ]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
