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
          "//modules/runtime:runtime",],
)

py_test(
  name = "scenario_generation_dump",
  srcs = ["scenario_generation_dump.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params',
          ':scenarios'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
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

filegroup(
    name="scenarios",
    srcs=glob(["scenarios/*.bark_scenarios"]),
)

filegroup(
    name="params",
    srcs=glob(["params/**"]),
)
