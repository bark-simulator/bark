filegroup(
  name = "notebooks_folder",
  srcs = glob(["*.ipynb"],
               exclude=["run.py", "run", "__init__.py"])
)

# add bark deps here
py_binary(
  name = "run",
  srcs = ["run.py"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",
          "@benchmark_database//load:benchmark_database",
          "@benchmark_database//serialization:database_serializer",
          "//modules/benchmark:benchmark_runner",
          "//modules/benchmark:benchmark_analyzer",],
  data = [":notebooks_folder",
          "//python:bark.so",
          "//modules/runtime/tests:xodr_data",
          "@benchmark_database//database:database",]
)

py_test(
  name = "notebook_tests",
  srcs = ["notebook_tests.py"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime"],
  data = [":notebooks_folder",
          "//python:bark.so",
          "//modules/runtime/tests:xodr_data"]
)