package(default_visibility = ["//visibility:public"])

py_test(
  name = "od8_const_vel_one_agent",
  srcs = ["od8_const_vel_one_agent.py"],
  data = ['//bark/pybark:core.so',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  #imports = ['../core/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/commons:model_json_conversion",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",],
)


py_binary(
  name = "analyze_map",
  srcs = ["analyze_map.py"],
  data = ['//bark/pybark:core.so',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  #imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",],
)


py_test(
  name = "planner_uct_benchmark",
  srcs = ["planner_uct_benchmark.py"],
  data = ['//bark/pybark:core.so',
          '@benchmark_database_release//:v2.0',
          ':params'],
  #imports = ['../python/'],
  deps = [
      "//bark/benchmark:benchmark_runner",
      "@benchmark_database//load:benchmark_database",
      "//bark/runtime/commons:parameters",
      ],
)

py_test(
  name = "planner_uct",
  srcs = ["planner_uct.py"],
  data = ['//bark/pybark:core.so',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  #imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:video_renderer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "od8_const_vel_two_agent",
  srcs = ["od8_const_vel_two_agent.py"],
  data = ['//bark/pybark:core.so',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  #imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/commons:model_json_conversion",
          "//bark/runtime:runtime"],
)

py_test(
  name = "scenario_video_rendering",
  srcs = ["scenario_video_rendering.py"],
  data = ['//bark/pybark:core.so',
          '//bark/runtime/tests:xodr_data',
          ':params',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/viewer:video_renderer",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "scenario_dump_load",
  srcs = ["scenario_dump_load.py"],
  data = ['//bark/pybark:core.so',
          '//bark/runtime/tests:xodr_data',
          ':params',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "scenarios_from_database",
  srcs = ["scenarios_from_database.py"],
  data = ['//bark/pybark:core.so',
          '@benchmark_database_release//:v2.0'],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime:runtime",
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
  data = ['//bark/pybark:core.so',
          "@bark_ml//tests/data:bark_data",
          '//bark/runtime/tests:xodr_data',
          '@benchmark_database_release//:v2.0',
          "@benchmark_database//data/database1:database1",
          "@bark_ml//bark/pybark:bark_ml.so",
          ':params',
          ':scenarios'],
  imports = ["../python/",
             "../external/bark_ml/python/"],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime",
          "//bark/models/behavior/plan:py_planners",
          "//bark/benchmark:benchmark_runner",
          "@benchmark_database//load:benchmark_database",
          "@benchmark_database//serialization:database_serializer",
          "@bark_ml//configurations/highway:configuration",
          "@bark_ml//configurations:bark_agent"],
)

py_binary(
    name = "interaction_dataset_runner",
    srcs = ["interaction_dataset_runner.py"],
    data = [
        ":params",
        "//bark/pybark:core.so",
    ] + 
    select({"//bark/runtime/scenario/interaction_dataset_processing:_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),
    imports = ["../python/"],
    deps = [
        "//bark/runtime/commons:parameters",
        "//bark/runtime/scenario/scenario_generation:scenario_generation",
        "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
        "//bark/runtime/viewer:matplotlib_viewer",
        "//bark/runtime/viewer:video_renderer",
        "//bark/runtime:runtime",
    ],
)

