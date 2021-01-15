package(default_visibility = ["//visibility:public"])

cc_library(
    name = "params",
    hdrs = [
        "params.hpp",
        "params_test.h",
        "setter_params.hpp",
        "params_c_api.h",
    ],
    srcs = [
        "setter_params.cpp",
        "params.cpp",
        "params_c_api.cpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//bark/commons/util:util",
        "//bark/commons/distribution:distribution",
        "@boost//:geometry", # add since sometimes "boost/variant.hpp" not found
    ]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)

