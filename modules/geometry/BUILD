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
        "//modules/commons:commons",
    ],
    visibility = ["//visibility:public"],
)
