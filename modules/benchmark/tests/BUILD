py_test(
  name = "py_benchmark_runner_tests",
  srcs = ["py_benchmark_runner_tests.py"],
  data = ['//python:bark.so',
          '@benchmark_database_release//:v1.0.0'],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner",
      "@benchmark_database//load:benchmark_database",
      "//modules/runtime/commons:parameters",
      ],
)