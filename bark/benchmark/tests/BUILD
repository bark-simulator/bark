package(default_visibility = ["//visibility:public"])
py_test(
  name = "py_benchmark_runner_tests",
  srcs = ["py_benchmark_runner_tests.py"],
  data = ["//bark:generate_core",
          "@benchmark_database//data/database1:database1",
          "//bark/runtime/tests:xodr_data"
          ],
  
  deps = [
      "//bark/benchmark:benchmark_runner",
      "//bark/benchmark:benchmark_runner_mp",
      "//bark/benchmark:benchmark_result",
      "//bark/runtime/viewer:matplotlib_viewer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//bark/runtime/commons:commons",
      ],
)

py_test(
  name = "py_benchmark_result_tests",
  srcs = ["py_benchmark_result_tests.py"],
  deps = [
      "//bark/benchmark:benchmark_result"
    ],
)

py_test(
  name = "py_benchmark_analyzer_tests",
  srcs = ["py_benchmark_analyzer_tests.py"],
  data = ["//bark:generate_core" ],
  deps = [
      "//bark/benchmark:benchmark_runner",
      "//bark/benchmark:benchmark_analyzer",
      "//bark/runtime/viewer:matplotlib_viewer",
      "//bark/runtime/commons:commons",
      ],
)


py_test(
  name = "py_benchmark_process_tests",
  srcs = ["py_benchmark_process_tests.py"],
  data = ["//bark:generate_core",
          "@benchmark_database//data/database1:database1",
          "//bark/runtime/tests:xodr_data"
          ],
  
  deps = [
      "//bark/benchmark:benchmark_runner_mp",
      "//bark/benchmark:benchmark_analyzer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//bark/runtime/viewer:matplotlib_viewer",
      "//bark/runtime/commons:commons",
      ],
)

