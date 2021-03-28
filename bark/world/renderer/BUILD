cc_library(
    name = "primitive",
    hdrs = ["primitive.hpp"],
    deps = ["//bark/geometry:geometry",
            "@com_github_eigen_eigen//:eigen"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "renderer",
    hdrs = ["renderer.hpp"],
    deps = ["//bark/world/renderer:primitive"],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)