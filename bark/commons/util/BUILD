
cc_library(
    name="util",
    hdrs=[
        "util.hpp",
        "operators.hpp",
        "segfault_handler.hpp"
    ],
    deps = [
        "@boost//:system",
        "@boost//:smart_ptr",
        "@boost//:move",
        "@boost//:type_traits",
        "@boost//:stacktrace",
        "@com_github_google_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=[
        "util.hpp",
        "operators.hpp"
    ],
    deps = [
        "@boost//:system",
        "@boost//:smart_ptr",
        "@boost//:move",
        "@boost//:type_traits",
        "@boost//:stacktrace",
        "@boost//:math",
        "@com_github_google_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)