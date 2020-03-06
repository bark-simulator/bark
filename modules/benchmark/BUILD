py_library(
    name = "benchmark_runner",
    srcs = ["benchmark_runner.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [
        "//modules/runtime:runtime",
        ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "benchmark_analyzer",
    srcs = ["benchmark_analyzer.py"],
    visibility = ["//visibility:public"]
)

py_library(
  name = "benchmark_runner_mp",
  srcs = ["benchmark_runner_mp.py"],
  data = ["//python:bark.so"],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner"
      ],
  visibility = ["//visibility:public"],
)

