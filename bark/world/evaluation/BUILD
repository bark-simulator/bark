cc_library(
    name = "base_evaluator",
    hdrs = [
        "base_evaluator.hpp",
    ],
    deps = [
        "//bark/geometry",
        "//bark/commons:commons",
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
        "//bark/world:world",
        ":base_evaluator",
        "@boost//:geometry",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_distance_to_goal",
    hdrs = [
        "evaluator_distance_to_goal.hpp",
    ],
    deps = [
        "//bark/world:world",
        ":base_evaluator",
        "@boost//:geometry",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_behavior_expired",
    hdrs = [
        "evaluator_behavior_expired.hpp",
    ],
    deps = [
        "//bark/world:world",
        "//bark/models/behavior:behavior",
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
        "//bark/world/evaluation:base_evaluator",
        "//bark/geometry",
        "//bark/commons:commons",
        "@boost//:geometry",
        "//bark/world:world",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_collision_ego_agent",
    hdrs = [
        "evaluator_collision_ego_agent.hpp"
    ],
    srcs = [
        "evaluator_collision_ego_agent.cpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/geometry",
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@boost//:geometry"
    ],
    visibility = ["//visibility:public"],
)
cc_library(
    name = "evaluator_step_count",
    hdrs = [
        "evaluator_step_count.hpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/geometry",
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@boost//:geometry"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_planning_time",
    hdrs = [
        "evaluator_planning_time.hpp"
    ],
    srcs = [
        "evaluator_planning_time.cpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/geometry",
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_velocity",
    hdrs = [
        "evaluator_velocity.hpp"
    ],
    srcs = [
        "evaluator_velocity.cpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_gap_distance_front",
    hdrs = [
        "evaluator_gap_distance_front.hpp"
    ],
    srcs = [
        "evaluator_gap_distance_front.cpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_number_of_agents",
    hdrs = [
        "evaluator_number_of_agents.hpp"
    ],
    srcs = [
        "evaluator_number_of_agents.cpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "commons",
    hdrs = [
        "commons.hpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/world:world",
        "//bark/models/dynamic:dynamic",
        "//bark/commons:commons"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_drivable_area",
    hdrs = [
        "evaluator_drivable_area.hpp"
    ],
    deps = [
        "//bark/world/evaluation:base_evaluator",
        "//bark/geometry",
        "@boost//:geometry",
        "//bark/world:world",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluation",
    deps = [
        ":evaluator_goal_reached",
        ":base_evaluator",
        ":evaluator_collision_agents",
        ":evaluator_drivable_area",
        ":evaluator_collision_ego_agent",
        ":evaluator_step_count",
        ":evaluator_behavior_expired",
        ":evaluator_distance_to_goal",
        ":evaluator_planning_time",
        ":evaluator_velocity",
        ":evaluator_gap_distance_front",
        ":evaluator_number_of_agents",
        ":commons",
        "//bark/world/evaluation/ltl:evaluator_ltl",
    ] + select({
        "//bark/world/evaluation/rss:_rss": [
            "//bark/world/evaluation/rss:evaluator_rss"
        ],
        "//conditions:default": []
    }),
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    deps = ["//bark/world/evaluation/ltl:include"],
    visibility = ["//visibility:public"],
)


