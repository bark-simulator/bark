
filegroup(
   name="xodr_data",
   srcs=glob(["data/**"]),
   visibility = ["//visibility:public"],
)

py_test(
  name = "py_importer_tests",
  srcs = ["py_importer_tests.py"],
  data = ['//python:bark.so',
          ':xodr_data'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime"],
)

py_test(
  name = "py_scenario_generation_tests",
  srcs = ["py_scenario_generation_tests.py"],
  data = ['//python:bark.so',
            ':xodr_data'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

py_test(
  name = "py_runtime_tests",
  srcs = ["py_runtime_tests.py"],
  data = ['//python:bark.so',
            ':xodr_data'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation"],
)

cc_library(
    name="py_param_server_test_helper",
    hdrs=["py_param_server_test_helper.hpp"],
    deps=[
        "//modules/commons:commons"
    ],
    visibility = ["//visibility:public"],
)

py_test(
  name = "py_param_server_tests",
  srcs = ["py_param_server_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../../python/'],
  deps = ["//modules/runtime/commons:parameters" ]
)

py_test(
  name = "py_evaluation_tests",
  srcs = ["py_evaluation_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime:runtime"],
)

py_test(
  name = "py_interaction_dataset_reader_test",
  srcs = ["py_interaction_dataset_reader_test.py"],
  data = ['//python:bark.so', ':xodr_data'],
  imports = ['../python/'],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime:runtime",
          "//modules/runtime/scenario/scenario_generation:scenario_generation",],
)
