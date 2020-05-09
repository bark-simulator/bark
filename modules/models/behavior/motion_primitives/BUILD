cc_library(
    name = "motion_primitives",
    srcs = [
        "continuous_actions.cpp",
        "macro_actions.cpp",
    ],
    hdrs = [
        "motion_primitives.hpp",
        "continuous_actions.hpp",
        "macro_actions.hpp",
        "primitives.hpp",
    ],
    deps = [
        "//modules/models/behavior/motion_primitives/param_config:param_config",
        "//modules/commons:commons",
        "//modules/world:include",
        "//modules/models/behavior:behavior",
        "//modules/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    deps = [
        "//modules/models/behavior/motion_primitives/param_config:include"
    ],
    visibility = ["//visibility:public"],
)
