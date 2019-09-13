cc_library(
    name = "viewer",
    hdrs = ["viewer.hpp",
            "viewer_functions.hpp"
    ],
    deps = [
      "//modules/commons:commons",
      "//modules/geometry:geometry",
	    "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"]
)

filegroup(
   name="model_data",
   srcs=glob(["models/**"]),
   visibility = ["//visibility:public"],
)

py_library(
    name = "py_viewer",
    srcs = ["viewer.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python/'],
    deps = ["//modules/runtime/commons:parameters",
            "//modules/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "threaded_viewer",
    srcs = ["threaded_viewer.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python/'],
    deps = ["//modules/runtime/commons:parameters",
            ":py_viewer",
            "//modules/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "video_renderer",
    srcs = ["video_renderer.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python/'],
    deps = ["//modules/runtime/commons:parameters",
            ":py_viewer",
            "//modules/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "pygame_viewer",
    srcs = ["pygame_viewer.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python/'],
    deps = ["//modules/runtime/commons:parameters",
            ":py_viewer",
            "//modules/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "panda3d_viewer",
    srcs = ["panda3d_viewer.py"],
    data = ['//python:bark.so',
            ':model_data'],
    imports = ['../../../python/'],
    deps = ["//modules/runtime/commons:parameters",
            ":py_viewer",
            "//modules/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "matplotlib_viewer",
    srcs = ["matplotlib_viewer.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python/'],
    deps = ["//modules/runtime/commons:parameters",
            ":py_viewer",
            "//modules/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)