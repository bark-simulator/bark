<<<<<<< HEAD
=======
py_test(
  name = "od8_const_vel_one_agent",
  srcs = ["od8_const_vel_one_agent.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/commons:model_json_conversion",
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
>>>>>>> Export classes from modules

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
          "//modules/runtime/viewer:panda3d_easy",
          "//modules/runtime/scenario/scenario_generation:scenario_generation",
          "//modules/runtime:runtime"],
)

py_test(
  name = "merging",
  srcs = ["merging.py"],
  data = ['//python:bark.so',
          '//modules/runtime/tests:xodr_data',
          ':params'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:pygame_viewer",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime/viewer:panda3d_easy",
          "//modules/runtime/scenario/scenario_generation:scenario_generation",
          "//modules/runtime:runtime"],
)

py_binary(
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
          "//modules/runtime/viewer:panda3d_easy",
          "//modules/runtime/scenario/scenario_generation:scenario_generation",
          "//modules/runtime:runtime"],
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
        "//python:bark.so",
        "//modules/runtime/tests:xodr_data"
    ] + 
    select({"//modules/runtime/scenario/interaction_dataset_processing:_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),
    imports = ["../python/"],
    deps = [
        "//modules/runtime/commons:parameters",
        "//modules/runtime/scenario/scenario_generation:scenario_generation",
        "//modules/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
        "//modules/runtime/viewer:matplotlib_viewer",
        "//modules/runtime/viewer:video_renderer",
        "//modules/runtime:runtime"
    ],
)


py_binary(
  name = "maude_interface",
  srcs = ["maude_interface.py"],
  data = ['//python:bark.so',
          '//modules/world/tests:params2'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime:runtime",
          "//modules/runtime/viewer:video_renderer"],
)

py_binary(
  name = "maude",
  srcs = ["maude.py"],
  data = ['//python:bark.so',
          '//modules/world/tests:params2'],
  imports = ['../python/'],
  deps = [':maude_interface'],
)