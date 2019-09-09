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
        ":driving_corridor",

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
    name = "driving_corridor",
    srcs = [
        "driving_corridor.cpp"
    ],
    hdrs = [
        "driving_corridor.hpp"
    ],
    deps = [
        "@boost//:geometry",
        ":frenet",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "frenet",
    srcs = [
        "frenet.cpp",
    ],
    hdrs = [
        "frenet.hpp",
    ],
    deps = [
        "//modules/geometry",
        "//modules/world/opendrive",
        "@boost//:geometry",
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
        "//modules/world/goal_definition:goal_definition",
        "//modules/world/map:frenet",
        "//modules/world/map:driving_corridor",
        "@boost//:geometry",
        "@boost//:graph",
        ":map_interface",
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