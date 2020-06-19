cc_library(
    name = "dynamic_model",
    srcs = [
        "dynamic_model.cpp",
    ],
    hdrs = [
        "dynamic_model.hpp",
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior:behavior",
        "//bark/models/dynamic:dynamic"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
