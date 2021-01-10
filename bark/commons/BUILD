package(default_visibility = ["//visibility:public"])

cc_library(
    name = "commons",
    hdrs = [
        "base_type.hpp",
        "commons.hpp"
    ],
    deps = [
        "//bark/commons/params:params",
        "//bark/commons/util:util",
        "//bark/commons/distribution:distribution"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "base_type",
    hdrs = [
        "base_type.hpp"
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    deps= [
        "//bark/commons/params:include",
        "//bark/commons/util:include",
        "//bark/commons/distribution:include",
        "@boost_bark",
        "@glog"
    ],
    visibility = ["//visibility:public"],
)
