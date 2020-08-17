filegroup(
  name = "notebooks_folder",
  srcs = glob(["*.ipynb"],
               exclude=["run.py", "run", "__init__.py"]),
  visibility = ["//visibility:public"],
)

# add bark deps here
py_binary(
  name = "run",
  srcs = [
         "run.py",
         "config_notebook.py"
         ],
  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime/viewer:matplotlib_viewer",
          "@benchmark_database//load:benchmark_database",
          "@benchmark_database//serialization:database_serializer",
          "//bark/benchmark:benchmark_runner",
          "//bark/benchmark:benchmark_analyzer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime"],
  data = [":notebooks_folder",
          "//bark:generate_core",
          "//bark/runtime/tests:xodr_data",
          "//examples:params",
          "@benchmark_database//data/database1:database1",
          ]
)

py_test(
  name = "notebook_tests",
  srcs = ["notebook_tests.py"],
  deps = ["//bark/runtime/commons:commons",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",
          "@benchmark_database//load:benchmark_database",
          "@benchmark_database//serialization:database_serializer",
          "//bark/benchmark:benchmark_runner",
          "//bark/benchmark:benchmark_analyzer",
          ],
  data = [":notebooks_folder",
          "//bark:generate_core",
          "//bark/runtime/tests:xodr_data",
          "@benchmark_database//data/database1:database1",
          ]
)