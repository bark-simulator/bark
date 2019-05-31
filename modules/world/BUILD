cc_library(
    name = "world",
    srcs = glob(["objects/*.cpp", "world*.cpp"]),
    hdrs = glob(["objects/*.hpp", "world*.hpp", "observed_world.hpp"]),
    deps = [
        "//modules/world/opendrive:opendrive",
        "//modules/world/map:roadgraph",
        "//modules/world/map:local_map",
        "//modules/commons:commons",
        "//modules/models/behavior:behavior",
        "//modules/models/execution:execution",
	    "//modules/models/dynamic:dynamic",
        "//modules/world/collision:base_collision_checker",
        "//modules/world/goal_definition:goal_definition"
            
    ],
    visibility = ["//visibility:public"],
)