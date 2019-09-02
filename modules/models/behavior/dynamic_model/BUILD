cc_library(
    name = "dynamic_model",
    srcs = [
        "dynamic_model.cpp",
    ],
    hdrs = [
        "dynamic_model.hpp",
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior:behavior",
        "//modules/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
