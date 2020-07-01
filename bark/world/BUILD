cc_library(
    name = "world",
    srcs = ["prediction/prediction_settings.cpp"] + glob(["objects/*.cpp", "world*.cpp", "observed_world.cpp"]),
    hdrs = ["prediction/prediction_settings.hpp"] + glob(["objects/*.hpp", "world*.hpp", "observed_world.hpp"]),
    deps = [
        "//bark/world/opendrive:opendrive",
        "//bark/world/map:roadgraph",
        "//bark/world/map:map_interface",
        "//bark/commons:commons",
        "//bark/commons/util:util",
        "//bark/commons/transformation:frenet",
        "//bark/models/behavior:behavior",
        "//bark/models/execution:execution",
        "//bark/models/dynamic:dynamic",
        "//bark/world/evaluation:base_evaluator",
        "//bark/world/goal_definition:goal_definition"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "include",
    hdrs = ["prediction/prediction_settings.hpp"] + glob(["objects/*.hpp", "world*.hpp", "observed_world.hpp"]),
    deps = [
        "//bark/geometry:include",
        "//bark/world/map:include",
        "//bark/world/opendrive:include",
        "//bark/commons/transformation:include",
        "//bark/commons:include",
        "//bark/models:include",
        "//bark/world/evaluation:include",
        "//bark/world/goal_definition:include",
    ],
    visibility = ["//visibility:public"],
)

