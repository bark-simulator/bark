py_library(
    name = "runtime",
    srcs = ["runtime.py"],
    data = ['//bark/core:core.so'],
    imports = ['../../../python'],
    deps = ["//bark/runtime/commons:parameters",
            "//bark/runtime/commons:commons",
            "//bark/runtime/viewer:pygame_viewer",
            "//bark/runtime/viewer:py_viewer",
            "//bark/runtime/viewer:panda3d_viewer",
            "//bark/runtime/viewer:matplotlib_viewer",
            "//bark/runtime/viewer:video_renderer",
            "//bark/runtime/commons:xodr_parser",
            "//bark/runtime/scenario:scenario"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cc_runtime",
    hdrs = [
        "runtime.hpp",
    ],
    deps = [
        "//bark/commons:commons",
    ],
    visibility = ["//visibility:public"],
)

#cpplint()