cc_library(
    name = "motion_primitives",
    srcs = [
        "continuous_actions.cpp",
        "macro_actions.cpp",
    ] + glob(["primitives/*.cpp"]),
    hdrs = [
        "motion_primitives.hpp",
        "continuous_actions.hpp",
        "macro_actions.hpp",
    ] + glob(["primitives/*.hpp"]),
    deps = [
        "//modules/commons:commons",
        "//modules/world:include",
        "//modules/world/map:road_corridor",
        "//modules/models/behavior:behavior",
        "//modules/models/dynamic:dynamic",
        "//modules/models/behavior:include",
        "//modules/models/behavior/motion_primitives/param_config:param_config",
        "//modules/models/behavior/idm:idm_lane_tracking",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["primitives/*.hpp"]) +
         glob(["*.hpp"]),
    deps = ["//modules/models/behavior/motion_primitives/param_config:include"],
    visibility = ["//visibility:public"],
)
