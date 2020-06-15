cc_library(
    name = "base_evaluator",
    hdrs = [
        "base_evaluator.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons",
        "//modules/geometry",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "evaluator_goal_reached",
    hdrs = [
        "evaluator_goal_reached.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":base_evaluator",
        "//modules/world",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "evaluator_behavior_expired",
    hdrs = [
        "evaluator_behavior_expired.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":base_evaluator",
        "//modules/models/behavior",
        "//modules/world",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "evaluator_collision_agents",
    srcs = [
        "evaluator_collision_agents.cpp",
    ],
    hdrs = [
        "evaluator_collision_agents.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons",
        "//modules/geometry",
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "evaluator_collision_ego_agent",
    srcs = [
        "evaluator_collision_ego_agent.cpp",
    ],
    hdrs = [
        "evaluator_collision_ego_agent.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons",
        "//modules/geometry",
        "//modules/models/dynamic",
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "evaluator_step_count",
    hdrs = [
        "evaluator_step_count.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons",
        "//modules/geometry",
        "//modules/models/dynamic",
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "evaluator_drivable_area",
    hdrs = [
        "evaluator_drivable_area.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/geometry",
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "rss_interface",
    srcs = ["rss_interface.cpp"],
    hdrs = ["rss_interface.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//modules/commons",
        "//modules/geometry",
        "//modules/models/dynamic",
        "//modules/world",
        "@ad_rss_lib//:ad_rss",
        "@ad_rss_lib//:ad_rss_map_integration",
        "@boost//:geometry",
        "@map_support//:ad_physics",
        "@spdlog",
    ],
)

cc_library(
    name = "evaluator_rss",
    hdrs = ["evaluator_rss.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":rss_interface",
        "//modules/world",
        "//modules/world/evaluation:base_evaluator",
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
    name = "include",
    hdrs = glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
