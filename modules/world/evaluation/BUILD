cc_library(
    name = "base_evaluator",
    hdrs = [
        "base_evaluator.hpp",
    ],
    deps = [
        "//modules/geometry",
        "//modules/commons:commons",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name = "evaluator_goal_reached",
    hdrs = [
        "evaluator_goal_reached.hpp",
    ],
    deps = [
        "//modules/world:world",
        ":base_evaluator",
        "@boost//:geometry",

    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name = "evaluator_collision_agents",
    hdrs = [
        "evaluator_collision_agents.hpp"
    ],
    srcs = [
        "evaluator_collision_agents.cpp"
    ],
    deps = [
        "//modules/world/evaluation:base_evaluator",
        "//modules/geometry",
        "//modules/commons:commons",
        "@boost//:geometry",
        "//modules/world:world",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_collision_driving_corridor",
    hdrs = [
        "evaluator_collision_driving_corridor.hpp"
    ],
    srcs = [
        "evaluator_collision_driving_corridor.cpp"
    ],
    deps = [
        "//modules/world/evaluation:base_evaluator",
        "//modules/geometry",
        "//modules/commons:commons",
        "@boost//:geometry",
        "//modules/world:world",

    ],
    visibility = ["//visibility:public"],
)



cc_library(
    name = "evaluation",
    deps = [
        ":evaluator_goal_reached",
        ":base_evaluator",
        ":evaluator_collision_agents",
        ":evaluator_collision_driving_corridor"
    ],
    visibility = ["//visibility:public"],
)


