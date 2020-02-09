package(default_visibility = ["//visibility:public"])

cc_library(
    name = "commons",
    hdrs = [
        "base_type.hpp"
    ],
    deps = [
        "//modules/commons/params:params",
        "//modules/commons/util:util"
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    deps= [
        "//modules/commons/params:include",
        "//modules/commons/util:include",
        "@boost//:system",
        "@boost//:smart_ptr",
        "@boost//:move",
        "@boost//:type_traits",
        "@boost//:stacktrace",
        "@com_github_google_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)
