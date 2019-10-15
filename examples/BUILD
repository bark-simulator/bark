py_binary(
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
  name = "planner_uct_benchmark",
  srcs = ["planner_uct_benchmark.py"],
  data = ['//python:bark.so',
          '@benchmark_database_release//:v1.0.0',
          ':params'],
  imports = ['../python/'],
  deps = [
      "//modules/benchmark:benchmark_runner",
      "@benchmark_database//load:benchmark_database",
      "//modules/runtime/commons:parameters",
      ],
)

py_binary(
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

py_binary(
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

py_binary(
  name = "scenario_generation_dump",
  srcs = ["scenario_generation_dump.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:threaded_viewer",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

py_binary(
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

py_binary(
  name = "drone_challenge",
  srcs = ["drone_challenge.py"],
  data = ['//python:bark.so',
          ':params',
          '//modules/runtime/tests:xodr_data',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime/viewer:video_renderer",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)


py_binary(
  name = "scenario_generation_load",
  srcs = ["scenario_generation_load.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':scenarios'],
  imports = ['../python/'],
  deps = [":scenario_generation_dump",
          "//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:pygame_viewer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

py_binary(
  name = "scenarios_from_database",
  srcs = ["scenarios_from_database.py"],
  data = ['//python:bark.so',
          '@benchmark_database_release//:v1.0.0'],
  imports = ['../python/'],
  deps = [":scenario_generation_dump",
          "//modules/runtime/commons:parameters",
          "//modules/runtime/viewer:matplotlib_viewer",
          "@benchmark_database//load:benchmark_database"],
)

filegroup(
    name="scenarios",
    srcs=glob(["scenarios/*.bark_scenarios"]),
)

filegroup(
    name="params",
    srcs=glob(["params/**"]),
)


py_binary(
  name = "barkml",
  srcs = ["barkml.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ["../python/"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation",
          "//modules/models/behavior/plan:py_planners"],
)
