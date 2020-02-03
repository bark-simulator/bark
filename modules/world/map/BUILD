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
        ":roadgraph",
        ":road_corridor",

    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "roadgraph",
    srcs = [
        "roadgraph.cpp",
    ],
    hdrs = [
        "roadgraph.hpp",
        "roadgraph_impl.hpp"
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
    name = "lane",
    srcs = [
        "lane.cpp"
    ],
    hdrs = [
        "lane.hpp"
    ],
    deps = [
        "//modules/world/opendrive"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "road",
    srcs = [
        "road.cpp"
    ],
    hdrs = [
        "road.hpp"
    ],
    deps = [
        "//modules/world/opendrive",
        ":lane"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "lane_corridor",
    srcs = [
        "lane_corridor.cpp"
    ],
    hdrs = [
        "lane_corridor.hpp"
    ],
    deps = [
        "//modules/world/opendrive",
        ":road",
        ":lane"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "road_corridor",
    srcs = [
        "road_corridor.cpp"
    ],
    hdrs = [
        "road_corridor.hpp"
    ],
    deps = [
        "//modules/world/opendrive",
        ":road",
        ":lane",
        ":lane_corridor"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["**/*.hpp"]),
    visibility = ["//visibility:public"],
    deps = [
        "@boost//:geometry",
        "@boost//:graph",
    ]
)