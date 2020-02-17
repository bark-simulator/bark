cc_library(
    name = "world",
    srcs = ["prediction/prediction_settings.cpp"] + glob(["objects/*.cpp", "world*.cpp", "observed_world.cpp"]),
    hdrs = ["prediction/prediction_settings.hpp"] + glob(["objects/*.hpp", "world*.hpp", "observed_world.hpp"]),
    deps = [
        "//modules/world/opendrive:opendrive",
        "//modules/world/map:roadgraph",
        "//modules/world/map:map_interface",
        "//modules/commons:commons",
        "//modules/commons/transformation:frenet",
        "//modules/models/behavior:behavior",
        "//modules/models/behavior/motion_primitives:motion_primitives",
        "//modules/models/execution:execution",
	    "//modules/models/dynamic:dynamic",
        "//modules/world/evaluation:base_evaluator",
        "//modules/world/goal_definition:goal_definition"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "include",
    hdrs = ["prediction/prediction_settings.hpp"] + glob(["objects/*.hpp", "world*.hpp", "observed_world.hpp"]),
    deps = [
        "//modules/geometry:include",
        "//modules/world/map:include",
        "//modules/world/opendrive:include",
        "//modules/commons/transformation:include",
        "//modules/commons:include",
        "//modules/models:include",
        "//modules/world/evaluation:include",
        "//modules/world/goal_definition:include"
    ],
    visibility = ["//visibility:public"],
)

