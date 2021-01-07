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
        "@boost_bark//:system",
        "@boost_bark//:smart_ptr",
        "@boost_bark//:move",
        "@boost_bark//:type_traits",
#        "@boost_bark//:stacktrace",
        "@boost_bark//:math",
        "@com_github_google_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)
