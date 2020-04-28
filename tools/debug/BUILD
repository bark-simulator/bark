py_binary(
  name = "analyze_map",
  srcs = ["analyze_map.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          '//examples:params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",],
)
