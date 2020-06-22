cc_library(
    name = "map_interface",
    srcs = [
        "map_interface.cpp"
    ],
    hdrs = [
        "map_interface.hpp"
    ],
    deps = [
        "//bark/geometry",
        "//bark/world/opendrive",
        "@boost//:geometry",
        ":roadgraph",
        ":road_corridor",

    ],
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
        "//bark/geometry",
        "//bark/world/opendrive",
        "@boost//:geometry",
        "@boost//:graph"
    ],
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
        "//bark/world/opendrive"
    ],
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
        "//bark/world/opendrive",
        ":lane"
    ],
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
        "//bark/world/opendrive",
        ":road",
        ":lane"
    ],
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
        "//bark/commons/transformation:frenet",
        "//bark/world/opendrive",
        ":road",
        ":lane",
        ":lane_corridor"
    ],
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