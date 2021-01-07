
cc_library(
    name="util",
    hdrs=[
        "util.hpp",
        "operators.hpp",
        "segfault_handler.hpp"
    ],
    deps = [
        "@boost_bark//:system",
        "@boost_bark//:smart_ptr",
        "@boost_bark//:move",
        "@boost_bark//:type_traits",
#        "@boost_bark//:stacktrace",
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
        "@boost_bark//:system",
        "@boost_bark//:smart_ptr",
        "@boost_bark//:move",
        "@boost_bark//:type_traits",
        "@boost_bark//:stacktrace",
        "@boost_bark//:math",
        "@com_github_google_glog//:glog"
    ],
    visibility = ["//visibility:public"],
)