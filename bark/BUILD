
genrule(
  name = "generate_core",
  srcs = [
    "//bark/python_wrapper:core.so",
  ],
  cmd = (" mv $(GENDIR)/bark/python_wrapper/core.so  $(location core.so)") ,
  #]),
  outs = [
    "core.so",
  ],
  visibility = ["//visibility:public"],
)

#bazel query 'filter(.tests.py, //...)'
genrule(
    name = "copy_tests",
    srcs = glob(["**/**/*.py"]),
    cmd = ("echo  $(SRCS) && mkdir -p $(GENDIR)/bark/tests  && cp -r $(SRCS) $(OUTS)"),
    outs=["tests"],
)


# filegroup(
#   name="test-files",
#   srcs = glob(
#         ["**/tests/*.py"],
#     ),
    
# )
# genrule(
#     name = "copy",
#     srcs = [":test-files"],
#     outs = ["test-files"],
#     cmd = "cp -r $(SRCS) $(OUTS)"
# )

py_test(
  name = "pip_package",
  srcs = ["pip_package.py"],
  data = [':generate_core',":copy_tests"],
  deps = ["//bark/runtime/commons:commons",
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
          "//bark/examples:planner_uct",],
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
