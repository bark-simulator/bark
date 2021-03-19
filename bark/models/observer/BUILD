cc_library(
    name = "observer_model",
    hdrs = [
        "observer_model.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "observer_model_none",
    hdrs = [
        "observer_model_none.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/models/observer:observer_model",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs= ["observer_model.hpp"],
    deps = [
        "//bark/models/observer:observer_model",
        "//bark/models/observer:observer_model_none",
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)
