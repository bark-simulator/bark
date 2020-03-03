py_test(
  name = "py_benchmark_runner_tests",
  srcs = ["py_benchmark_runner_tests.py"],
  data = ["//python:bark.so",
          "@benchmark_database//database:database",
          "//modules/runtime/tests:xodr_data"
          ],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner",
      "//modules/benchmark:benchmark_runner_mp",
      "//modules/runtime/viewer:matplotlib_viewer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//modules/runtime/commons:parameters",
      ],
)

py_test(
  name = "py_benchmark_analyzer_tests",
  srcs = ["py_benchmark_analyzer_tests.py"],
  data = ["//python:bark.so",
          ],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner",
      "//modules/benchmark:benchmark_analyzer",
      "//modules/runtime/viewer:matplotlib_viewer",
      "//modules/runtime/commons:parameters",
      ],
)


py_test(
  name = "py_benchmark_process_tests",
  srcs = ["py_benchmark_process_tests.py"],
  data = ["//python:bark.so",
        "@benchmark_database//database:database",
          "//modules/runtime/tests:xodr_data"
          ],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner_mp",
      "//modules/benchmark:benchmark_analyzer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//modules/runtime/viewer:matplotlib_viewer",
      "//modules/runtime/commons:parameters",
      ],
)

