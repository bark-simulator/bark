
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
        "@glog"
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
        "@glog"
    ],
    visibility = ["//visibility:public"],
)