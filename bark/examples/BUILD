package(default_visibility = ["//visibility:public"])

py_test(
  name = "benchmark_database",
  srcs = ["benchmark_database.py"],
  data = ["//bark:generate_core",
          '@benchmark_database_release//:v2.0',
          ':params'],
  deps = [
      "//bark/benchmark:benchmark_runner",
      "@benchmark_database//load:benchmark_database",
      "//bark/runtime/commons:parameters",
      ],
)

py_test(
  name = "highway",
  srcs = ["highway.py"],
  data = ["//bark:generate_core",
          '//bark/runtime/tests:xodr_data',
          ':params'],

  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths"],
)

py_test(
  name = "merging",
  srcs = ["merging.py"],
  data = ["//bark:generate_core",
          '//bark/runtime/tests:xodr_data',
          ':params'],

  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/viewer:buffered_mp_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths"],
)

py_test(
  name = "parametric_observer",
  srcs = ["parametric_observer.py"],
  data = ["//bark:generate_core",
          '//bark/runtime/tests:xodr_data',
          ':params'],

  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths"],
)

py_binary(
  name = "intersection",
  srcs = ["intersection.py"],
  data = ["//bark:generate_core",
          '//bark/runtime/tests:xodr_data',
          ':params'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths"],
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


py_binary(
    name = "interaction_dataset",
    srcs = ["interaction_dataset.py"],
    data = [
        ":params",
        "//bark:generate_core",
        "//bark/runtime/tests:xodr_data"
    ] +
    select({"//bark/runtime/scenario/interaction_dataset_processing:_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),

    deps = [
        "//bark/runtime/commons:parameters",
        "//bark/runtime/scenario/scenario_generation:scenario_generation",
        "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
        "//bark/runtime/viewer:matplotlib_viewer",
        "//bark/runtime/viewer:video_renderer",
        "//bark/runtime:runtime",
        ":paths"],
)

py_binary(
  name = "highway_rss",
  srcs = ["highway_rss.py"],
  data = ["//bark:generate_core",
          '//bark/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths"],
)

py_binary(
  name = "merging_rss",
  srcs = ["merging_rss.py"],
  data = ["//bark:generate_core",
          '//bark/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths"],
)

py_binary(
  name = "barkscape_example",
  srcs = ["barkscape_example.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/viewer:buffered_viewer",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime",
          ":paths",
          "@barkscape_project//barkscape/server/runners:bark_runner",
          "@barkscape_project//barkscape/server:bark_xviz_stream",
          "@barkscape_project//barkscape/server:base_server",
          "@barkscape_project//barkscape/server:base_handler"],
)

py_library(
  name = "paths",
  srcs = ["paths.py"],
)