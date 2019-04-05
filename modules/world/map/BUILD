cc_library(
    name = "map_interface",
    srcs = [
        "map_interface.cpp"
    ],
    hdrs = [
        "map_interface.hpp"
    ],
    deps = [
        "//modules/geometry",
        "//modules/world/opendrive",
        "@boost//:geometry",
        ":roadgraph"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "roadgraph",
    srcs = [
    ],
    hdrs = [
        "roadgraph.hpp",
    ],
    deps = [
        "//modules/geometry",
        "//modules/world/opendrive",
        "@boost//:geometry",
        "@boost//:graph"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "local_map",
    srcs = [
        "local_map.cpp"
    ],
    hdrs = [
        "local_map.hpp",
    ],
    deps = [
        "//modules/geometry",
        "//modules/world/opendrive",
        "@boost//:geometry",
        "@boost//:graph",
        ":map_interface"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)