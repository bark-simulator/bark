py_test(
  name = "py_benchmark_runner_tests",
  srcs = ["py_benchmark_runner_tests.py"],
  data = ["//python:bark.so",
          "@benchmark_database//database:database"],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//modules/runtime/commons:parameters",
      ],
)