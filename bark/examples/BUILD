package(default_visibility = ["//visibility:public"])

py_test(
  name = "benchmark_database",
  srcs = ["benchmark_database.py"],
  data = ['//bark:generate_core',
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
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/viewer:panda3d_easy",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime"],
)

py_test(
  name = "merging",
  srcs = ["merging.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/viewer:panda3d_easy",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime"],
)

py_binary(
  name = "intersection",
  srcs = ["intersection.py"],
  data = ['//bark:generate_core',
          '//bark/runtime/tests:xodr_data',
          ':params'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/viewer:pygame_viewer",
          "//bark/runtime/viewer:matplotlib_viewer",
          "//bark/runtime/viewer:panda3d_easy",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          "//bark/runtime:runtime"],
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
        "//bark/runtime:runtime"
    ],
)


py_binary(
  name = "maude_interface",
  srcs = ["maude_interface.py"],
  data = ['//bark:generate_core',
          '//bark/world/tests:params2'],
 
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime",
          "//bark/runtime/viewer:video_renderer"],
)

py_binary(
  name = "maude",
  srcs = ["maude.py"],
  data = ['//bark:generate_core',
          '//bark/world/tests:params2'],
  
  deps = [':maude_interface'],
)