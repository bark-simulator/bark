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
        "//bark/commons/distribution:distribution",
        "//bark/commons/math:math",
        "//bark/commons/timer:timer",
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
        "//bark/commons/math:include",
        "//bark/commons/timer:include",
        "@boost//:system",
        "@boost//:smart_ptr",
        "@boost//:move",
        "@boost//:type_traits",
        "@boost//:stacktrace",
        "@boost//:math",
        "@com_github_glog_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)
