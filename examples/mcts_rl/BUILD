
py_test(
  name = "run_benchmark",
  srcs = ["run_benchmark.py"],
  data = ['//bark:generate_core',
          '//examples/mcts_rl/database:database',
          ':params'],
  imports = ['../python/'],
  deps = [
      "//bark/benchmark:benchmark_runner",
      "//bark/runtime/commons:parameters",
        "//bark/benchmark:benchmark_runner_mp",
      "//bark/benchmark:benchmark_analyzer",
      "@benchmark_database//load:benchmark_database",
      "@benchmark_database//serialization:database_serializer",
      "//bark/runtime/viewer:matplotlib_viewer",
      ],
)


filegroup(
    name="params",
    srcs=glob(["params/**"]),
    visibility = ["//visibility:public"],
)


