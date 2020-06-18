py_library(
    name = "runtime",
    srcs = ["runtime.py"],
    data = ['//bark:generate_core'],
    imports = ['../../../python'],
    deps = ["//bark/runtime/commons:commons",
            "//bark/runtime/viewer:pygame_viewer",
            "//bark/runtime/viewer:py_viewer",
            "//bark/runtime/viewer:panda3d_viewer",
            "//bark/runtime/viewer:matplotlib_viewer",
            "//bark/runtime/viewer:video_renderer",
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