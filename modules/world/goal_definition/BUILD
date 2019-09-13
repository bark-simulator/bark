cc_library(
    name = "goal_definition",
    srcs = [
        "goal_definition_polygon.cpp"
    ],
    hdrs = [
        "goal_definition.hpp",
        "goal_definition_polygon.hpp"
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
