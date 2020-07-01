cc_library(
    name = "geometry",
    hdrs = [
        "polygon.hpp",
        "line.hpp",
        "commons.hpp",
        "standard_shapes.hpp",
        "model_3d.hpp",
        "geometry.hpp",
        "angle.hpp"
    ],
    srcs = [
        "standard_shapes.cpp"
    ],
    deps = [
	    "@com_github_eigen_eigen//:eigen",
        "@boost//:geometry",
        "//bark/commons:commons",
    ],
    visibility = ["//visibility:public"],
)


cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    deps = [
	    "@com_github_eigen_eigen//:eigen",
        "@boost//:geometry",
    ],
    visibility = ["//visibility:public"],
)