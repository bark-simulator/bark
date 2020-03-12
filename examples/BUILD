py_test(
  name = "od8_const_vel_one_agent",
  srcs = ["od8_const_vel_one_agent.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:pygame_viewer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",],
)

py_binary(
  name = "analyze_map",
  srcs = ["analyze_map.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",],
)

py_test(
  name = "planner_uct_benchmark",
  srcs = ["planner_uct_benchmark.py"],
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
  name = "planner_uct",
  srcs = ["planner_uct.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:video_renderer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "od8_const_vel_two_agent",
  srcs = ["od8_const_vel_two_agent.py"],
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
  name = "scenario_video_rendering",
  srcs = ["scenario_video_rendering.py"],
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

py_test(
  name = "scenario_dump_load",
  srcs = ["scenario_dump_load.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:pygame_viewer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "scenarios_from_database",
  srcs = ["scenarios_from_database.py"],
  data = ['//python:bark.so',
          '@benchmark_database_release//:v2.0'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",
          "@benchmark_database//load:benchmark_database"],
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
  name = "barkml",
  srcs = ["barkml.py"],
  data = ['//python:bark.so',
          "@bark_ml//tests/data:bark_data",
          '//modules/runtime/tests:xodr_data',
          '@benchmark_database_release//:v2.0',
          "@benchmark_database//database:database",
          "@bark_ml//python:bark_ml.so",
          ':params',
          ':scenarios'],
  imports = ["../python/",
             "../external/bark_ml/python/"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime:runtime",
          "//modules/models/behavior/plan:py_planners",
          "//modules/benchmark:benchmark_runner",
          "@benchmark_database//load:benchmark_database",
          "@benchmark_database//serialization:database_serializer",
          "@bark_ml//configurations/highway:configuration",
          "@bark_ml//configurations:bark_agent"],
)

config_setting(
    name = "_interaction_dataset_present",
    define_values = {"interaction_dataset_present": "true"},
)

py_binary(
    name = "interaction_dataset_runner",
    srcs = ["interaction_dataset_runner.py"],
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