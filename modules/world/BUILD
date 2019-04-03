cc_library(
    name = "world",
    srcs = glob(["objects/*.cpp", "world*.cpp"]),
    hdrs = glob(["objects/*.hpp", "world*.hpp", "observed_world.hpp"]),
    deps = [
        "//modules/world/opendrive:opendrive",
        "//modules/world/map:roadgraph",
        "//modules/world/map:route_generator",
        "//modules/commons:commons",
        "//modules/models/behavior:behavior",
        "//modules/models/execution:execution",
		    "//modules/models/dynamic:dynamic",
    ],
    visibility = ["//visibility:public"],
)