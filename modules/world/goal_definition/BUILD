cc_library(
    name = "goal_definition",
    srcs = [
        "goal_definition_polygon.cpp",
        "goal_definition_state_limits.cpp",
        "goal_definition_sequential.cpp"
    ],
    hdrs = [
        "goal_definition.hpp",
        "goal_definition_polygon.hpp",
        "goal_definition_state_limits.hpp",
        "goal_definition_sequential.hpp"
    ],
    deps = [
        "//modules/geometry",
        "//modules/world:include",
        "@boost//:geometry",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
