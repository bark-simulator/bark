py_library(
    name = "benchmark_runner",
    srcs = ["benchmark_runner.py"],
    data = ['//bark:generate_core'],
    imports = ['../../../python'],
    deps = [
        "//bark/runtime:runtime",
        ":benchmark_result"
        ],
    visibility = ["//visibility:public"],
)

py_library(
    name = "benchmark_result",
    srcs = ["benchmark_result.py"],
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
  data = ["//bark:generate_core"],
  deps = [
      "//bark/benchmark:benchmark_runner"
      ],
  visibility = ["//visibility:public"],
)

