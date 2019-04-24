cc_library(
    name = "base_collision_checker",
    hdrs = [
        "base_collision_checker.hpp"
    ],
    deps = [
        "//modules/geometry",
        "//modules/commons:commons",
        "@boost//:geometry",

    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "collision_checker_agents",
    hdrs = [
        "collision_checker_agents.hpp"
    ],
    srcs = [
        "collision_checker_agents.cpp"
    ],
    deps = [
        "//modules/world/collision:base_collision_checker",
        "//modules/geometry",
        "//modules/commons:commons",
        "@boost//:geometry",
        "//modules/world:world",

    ],
    visibility = ["//visibility:public"],
)

