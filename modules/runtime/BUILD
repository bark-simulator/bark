py_library(
    name = "runtime",
    srcs = ["runtime.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = ["//modules/runtime/commons:parameters",
            "//modules/runtime/commons:commons",
            "//modules/runtime/viewer:pygame_viewer",
            "//modules/runtime/viewer:py_viewer",
            "//modules/runtime/viewer:panda3d_viewer",
            "//modules/runtime/viewer:matplotlib_viewer"],
    visibility = ["//visibility:public"],
)