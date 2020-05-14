py_test(
  name = "py_benchmark_runner_tests",
  srcs = ["py_benchmark_runner_tests.py"],
  data = ["//bark/pybark:core.so",
          "@benchmark_database//data/database1:database1",
          "//bark/runtime/tests:xodr_data"
          ],
  imports = ['../python/'],
  deps = [
      "//bark/benchmark:benchmark_runner",
      "//bark/benchmark:benchmark_runner_mp",
      "//bark/runtime/viewer:matplotlib_viewer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//bark/runtime/commons:parameters",
      ],
)

py_test(
  name = "py_benchmark_analyzer_tests",
  srcs = ["py_benchmark_analyzer_tests.py"],
  data = ["//bark/pybark:core.so",
          ],
  imports = ['../python/'],
  deps = [
      "//bark/benchmark:benchmark_runner",
      "//bark/benchmark:benchmark_analyzer",
      "//bark/runtime/viewer:matplotlib_viewer",
      "//bark/runtime/commons:parameters",
      ],
)


py_test(
  name = "py_benchmark_process_tests",
  srcs = ["py_benchmark_process_tests.py"],
  data = ["//bark/pybark:core.so",
          "@benchmark_database//data/database1:database1",
          "//bark/runtime/tests:xodr_data"
          ],
  imports = ['../python/'],
  deps = [
      "//bark/benchmark:benchmark_runner_mp",
      "//bark/benchmark:benchmark_analyzer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//bark/runtime/viewer:matplotlib_viewer",
      "//bark/runtime/commons:parameters",
      ],
)

