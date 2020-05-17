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
    name = "evaluator_behavior_expired",
    hdrs = [
        "evaluator_behavior_expired.hpp",
    ],
    deps = [
        "//modules/world:world",
        "//modules/models/behavior:behavior",
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
    name = "evaluator_collision_ego_agent",
    hdrs = [
        "evaluator_collision_ego_agent.hpp"
    ],
    srcs = [
        "evaluator_collision_ego_agent.cpp"
    ],
    deps = [
        "//modules/world/evaluation:base_evaluator",
        "//modules/world:world",
        "//modules/geometry",
        "//modules/commons:commons",
        "//modules/models/dynamic:dynamic",
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
        "//modules/world/evaluation:base_evaluator",
        "//modules/world:world",
        "//modules/geometry",
        "//modules/commons:commons",
        "//modules/models/dynamic:dynamic",
        "@boost//:geometry"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_drivable_area",
    hdrs = [
        "evaluator_drivable_area.hpp"
    ],
    deps = [
        "//modules/world/evaluation:base_evaluator",
        "//modules/geometry",
        "@boost//:geometry",
        "//modules/world:world",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_rss",
    srcs = ["evaluator_rss.cpp"],
    hdrs = ["evaluator_rss.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons",
        "//modules/geometry",
        "//modules/models/dynamic",
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
        "@ad_rss_lib//:ad_rss",
        "@ad_rss_lib//:ad_rss_map_integration",
        "@boost//:geometry",
        "@spdlog",
    ],
)

cc_library(
    name = "evaluation",
    visibility = ["//visibility:public"],
    deps = [
        ":base_evaluator",
        ":evaluator_behavior_expired",
        ":evaluator_collision_agents",
        ":evaluator_collision_ego_agent",
        ":evaluator_drivable_area",
        ":evaluator_goal_reached",
        ":evaluator_rss",
        ":evaluator_step_count",
    ],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)


