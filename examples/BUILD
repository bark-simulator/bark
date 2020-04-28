
py_test(
  name = "benchmark_database",
  srcs = ["benchmark_database.py"],
  data = ['//python:bark.so',
          '@benchmark_database_release//:v2.0',
          ':params'],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner",
      "@benchmark_database//load:benchmark_database",
      "//modules/runtime/commons:parameters",
      ],
)

py_test(
  name = "highway",
  srcs = ["highway.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:pygame_viewer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime/scenario/scenario_generation:scenario_generation",
          "//modules/runtime:runtime"],
)

py_test(
  name = "intersection",
  srcs = ["intersection.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:pygame_viewer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime"],
)

py_test(
  name = "video_rendering",
  srcs = ["video_rendering.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime/viewer:video_renderer",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

filegroup(
    name="scenarios",
    srcs=glob(["scenarios/**"]),
)

filegroup(
    name="params",
    srcs=glob(["params/**"]),
    visibility = ["//visibility:public"],
)

config_setting(
    name = "_interaction_dataset_present",
    define_values = {"interaction_dataset_present": "true"},
)

py_binary(
    name = "interaction_dataset",
    srcs = ["interaction_dataset.py"],
    data = [
        ":params",
        "//python:bark.so",
    ] + 
    select({":_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),
    imports = ["../python/"],
    deps = [
        "//modules/runtime/commons:parameters",
        "//modules/runtime/scenario/scenario_generation",
        "//modules/runtime/viewer:matplotlib_viewer",
        "//modules/runtime/viewer:video_renderer",
        "//modules/runtime:runtime",
    ],
)