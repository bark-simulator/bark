py_library(
    name = "benchmark_runner",
    srcs = ["benchmark_runner.py"],
    data = ['//bark/core:core.so'],
    imports = ['../../../python'],
    deps = [
        "//bark/runtime:runtime",
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
  data = ["//bark/core:core.so"],
  imports = ['../python/'],
  deps = [
      "//bark/benchmark:benchmark_runner"
      ],
  visibility = ["//visibility:public"],
)

