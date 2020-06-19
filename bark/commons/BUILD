package(default_visibility = ["//visibility:public"])

cc_library(
    name = "commons",
    hdrs = [
        "base_type.hpp"
    ],
    deps = [
        "//bark/commons/params:params",
        "//bark/commons/util:util"
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    deps= [
        "//bark/commons/params:include",
        "//bark/commons/util:include",
        "@boost//:system",
        "@boost//:smart_ptr",
        "@boost//:move",
        "@boost//:type_traits",
        "@boost//:stacktrace",
        "@com_github_google_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)
