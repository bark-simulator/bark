filegroup(
  name = "notebooks_folder",
  srcs = glob(["*.ipynb"])
)

# add bark deps here
py_binary(
  name = "run",
  srcs = ["run.py"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime"],
  data = [":notebooks_folder",
          "//python:bark.so",
          "//modules/runtime/tests:xodr_data"]
)

py_test(
  name = "notebooks_tests",
  srcs = ["notebooks_tests.py"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime"],
  data = [":notebooks_folder",
          "//python:bark.so",
          "//modules/runtime/tests:xodr_data"]
)