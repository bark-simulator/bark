
filegroup(
   name="xodr_data",
   srcs=glob(["data/**"]),
   visibility = ["//visibility:public"],
)

filegroup(
   name="track_data",
   srcs=glob(["data/*.csv"]),
   visibility = ["//visibility:public"],
)

py_test(
  name = "py_importer_tests",
  srcs = ["py_importer_tests.py"],
  data = ['//bark/core:core.so',
          ':xodr_data'],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime:runtime"],
)

py_test(
  name = "py_scenario_generation_tests",
  srcs = ["py_scenario_generation_tests.py"],
  data = ['//bark/core:core.so',
            ':xodr_data'],
  imports = ['../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
          "//bark/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "py_runtime_tests",
  srcs = ["py_runtime_tests.py"],
  data = ['//bark/core:core.so',
            ':xodr_data'],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime/commons:xodr_parser",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/scenario_generation:scenario_generation"],
)

cc_library(
    name="py_param_server_test_helper",
    hdrs=["py_param_server_test_helper.hpp"],
    deps=[
        "//bark/commons:commons"
    ],
    visibility = ["//visibility:public"],
)

py_test(
  name = "py_param_server_tests",
  srcs = ["py_param_server_tests.py"],
  data = ['//bark/core:core.so'],
  imports = ['../../../python/'],
  deps = ["//bark/runtime/commons:parameters" ]
)

py_test(
  name = "py_evaluation_tests",
  srcs = ["py_evaluation_tests.py"],
  data = ['//bark/core:core.so'],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime"],
)

py_test(
  name = "py_interaction_dataset_reader_test",
  srcs = ["py_interaction_dataset_reader_test.py"],
  data = ['//bark/core:core.so', 
          ':xodr_data',
         ],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
          ],
)


py_test(
  name = "py_interaction_dataset_decomposer_test",
  srcs = ["py_interaction_dataset_decomposer_test.py"],
  data = ['//bark/core:core.so', 
          ':xodr_data',
          ':track_data',
         ],
  imports = ['../python/'],
  deps = ["//bark/runtime/commons:parameters",
          "//bark/runtime:runtime",
          "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing",
          "//bark/runtime/scenario/scenario_generation:scenario_generation",
          ],
)
