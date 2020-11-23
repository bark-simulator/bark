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

filegroup(
   name="xml_template",
   srcs=glob(["*.xml"]),
   visibility = ["//visibility:public"],
)

py_library(
  name = "scenario_dumper",
  srcs = ["scenario_dumper.py"],
  data = ["//bark:generate_core",
          ":xml_template"],
  deps = [
      "//bark/runtime/commons:commons",
      "//bark/runtime/viewer:matplotlib_viewer",
      "//bark/benchmark:benchmark_analyzer",
      "//bark/benchmark:benchmark_result",
      ],
  visibility = ["//visibility:public"],
)



