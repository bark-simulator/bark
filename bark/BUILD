# py_library(
#   name = "init_py",
#   srcs = ["__init__.py"],
#   visibility = ["//visibility:public"],
# )


genrule(
  name = "movelib",
  srcs = [
    "//bark/pybark:core.so",
  ],
  cmd = (" mv $(GENDIR)/bark/pybark/core.so  $(location core.so)") ,
  #]),
  outs = [
    "core.so",
  ],
  visibility = ["//visibility:public"],
)


# sh_binary(
#     name = "movelib",
#     srcs = ["movelib.sh"],
#     data = ["//bark/pybark:core.so"],
# )

py_test(
  name = "pip_package",
  srcs = ["pip_package.py"],
  data = [':movelib'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:commons",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/commons:model_json_conversion",
          "//bark/runtime/scenario:scenario",
          "//bark/runtime/viewer:py_viewer",
          "//bark/runtime/viewer:threaded_viewer",
          "//bark/runtime/viewer:video_renderer",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:panda3d_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",
          "//bark/examples:od8_const_vel_one_agent",
          "//bark/examples:od8_const_vel_two_agent",
          "//bark/examples:planner_uct",
          ],
)

cc_library(
    name="include",
    deps=[
        "//bark/commons:include",
        "//bark/geometry:include",
        "//bark/models:include",
        "//bark/world:include"
    ],
    visibility = ["//visibility:public"],
)
