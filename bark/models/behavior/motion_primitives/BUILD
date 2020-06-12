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
        "//bark/commons:commons",
        "//bark/world:include",
        "//bark/world/map:road_corridor",
        "//bark/models/behavior:behavior",
        "//bark/models/dynamic:dynamic",
        "//bark/models/behavior:include",
        "//bark/models/behavior/motion_primitives/param_config:param_config",
        "//bark/models/behavior/idm:idm_lane_tracking",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["primitives/*.hpp"]) +
         glob(["*.hpp"]),
    deps = ["//bark/models/behavior/motion_primitives/param_config:include"],
    visibility = ["//visibility:public"],
)
