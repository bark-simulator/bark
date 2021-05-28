cc_library(
    name = "viewer",
    hdrs = ["viewer.hpp",
            "viewer_functions.hpp"
    ],
    deps = [
      "//bark/commons:commons",
      "//bark/geometry:geometry",
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
    srcs = ["viewer.py","__init__.py"],
    data = ["//bark:generate_core"],
    deps = ["//bark/runtime/commons:commons"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "threaded_viewer",
    srcs = ["threaded_viewer.py"],
    data = ["//bark:generate_core"],
    deps = ["//bark/runtime/commons:commons",
            ":py_viewer"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "video_renderer",
    srcs = ["video_renderer.py"],
    data = ["//bark:generate_core"],
    deps = ["//bark/runtime/commons:commons",
            ":py_viewer"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "pygame_viewer",
    srcs = ["pygame_viewer.py"],
    data = ["//bark:generate_core"],
    deps = ["//bark/runtime/commons:commons",
            ":py_viewer"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "matplotlib_viewer",
    srcs = ["matplotlib_viewer.py"],
    data = ["//bark:generate_core"],
    deps = ["//bark/runtime/commons:commons",
            ":py_viewer"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "buffered_viewer",
    srcs = ["buffered_viewer.py"],
    data = ["//bark:generate_core"],
    deps = ["//bark/runtime/commons:commons",
            ":py_viewer"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "buffered_mp_viewer",
    srcs = ["buffered_mp_viewer.py"],
    data = ['//bark:generate_core'],
    deps = ["//bark/runtime/commons:commons",
            ":py_viewer",
            ":buffered_viewer"],
    visibility = ["//visibility:public"],
)
