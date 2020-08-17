cc_library(
    name = "behavior",
    hdrs = [
        "behavior_model.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/models/dynamic:dynamic",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs= ["behavior_model.hpp"],
    deps = [
        "//bark/models/behavior/constant_acceleration:include",
        "//bark/models/behavior/dynamic_model:include",
        "//bark/models/behavior/idm:include",
        "//bark/models/behavior/rule_based:include",
        "//bark/models/behavior/motion_primitives:include",
        "//bark/models/behavior/static_trajectory:include",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

#cpplint()
