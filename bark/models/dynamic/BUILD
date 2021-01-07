cc_library(
    name = "dynamic",
    hdrs = [
        "integration.hpp",
        "single_track.hpp",
        "dynamic_model.hpp",
        "triple_integrator.hpp",
    ],
    deps = [
        "//bark/commons:commons",
	    "@com_github_eigen_eigen//:eigen",
        "@boost_bark//:geometry",
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)