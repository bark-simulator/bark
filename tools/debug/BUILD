py_binary(
  name = "analyze_map",
  srcs = [
        "analyze_map.py",
        "debug_settings.py"
        ],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data',
          '//examples:params'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",],
)

py_binary(
  name = "debug_settings",
  srcs = [
        "debug_settings.py"
        ],
  visibility = ["//visibility:public"],
)
