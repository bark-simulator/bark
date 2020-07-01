cc_library(
    name = "goal_definition",
    srcs = [
        "goal_definition_polygon.cpp",
        "goal_definition_state_limits.cpp",
        "goal_definition_sequential.cpp",
        "goal_definition_state_limits_frenet.cpp"
    ],
    hdrs = [
        "goal_definition.hpp",
        "goal_definition_polygon.hpp",
        "goal_definition_state_limits.hpp",
        "goal_definition_sequential.hpp"
    ],
    deps = [
        "//bark/geometry",
        "//bark/world:include",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
