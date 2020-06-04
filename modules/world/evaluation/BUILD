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
    name = "evaluator_distance_to_goal",
    hdrs = [
        "evaluator_distance_to_goal.hpp",
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
    name = "evaluator_ltl",
    hdrs = [
        "evaluator_ltl.hpp"
    ],
    srcs = [
         "evaluator_ltl.cpp",
    ],
    deps = [
        "//modules/world/evaluation:base_evaluator",
        "//modules/world:world",
        "@rule_monitor_project//ltl:rule_monitor"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_right_overtake",
    srcs = ["evaluator_right_overtake.cpp"],
    hdrs = ["evaluator_right_overtake.hpp"],
    deps = [
        ":evaluator_ltl",
        "//modules/world/evaluation/labels:label_evaluation"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_safe_distance",
    srcs = ["evaluator_safe_distance.cpp"],
    hdrs = ["evaluator_safe_distance.hpp"],
    deps = [
        ":evaluator_ltl",
        "//modules/world/evaluation/labels:label_evaluation"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_safe_lane_change",
    srcs = ["evaluator_safe_lane_change.cpp"],
    hdrs = ["evaluator_safe_lane_change.hpp"],
    deps = [
        ":evaluator_ltl",
        "//modules/world/evaluation/labels:label_evaluation"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_being_overtaken",
    srcs = ["evaluator_being_overtaken.cpp"],
    hdrs = ["evaluator_being_overtaken.hpp"],
    deps = [
        ":evaluator_ltl",
        "//modules/world/evaluation/labels:label_evaluation"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "evaluator_rel_speed_overtake",
    srcs = ["evaluator_rel_speed_overtake.cpp"],
    hdrs = ["evaluator_rel_speed_overtake.hpp"],
    deps = [
        ":evaluator_ltl",
        "//modules/world/evaluation/labels:label_evaluation"
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
        ":evaluator_ltl",
        ":evaluator_right_overtake",
        ":evaluator_safe_distance",
        ":evaluator_safe_lane_change",
        ":evaluator_being_overtaken",
        ":evaluator_rel_speed_overtake",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)


