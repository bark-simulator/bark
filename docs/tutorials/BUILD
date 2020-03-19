filegroup(
  name = "notebooks_folder",
  srcs = glob(["*.ipynb"])
)

# add bark deps here
py_binary(
  name = "run",
  srcs = ["run.py"],
  deps = ["//modules/runtime/commons:parameters",
          "//modules/runtime/commons:xodr_parser",
          "//modules/runtime/viewer:matplotlib_viewer",
          "//modules/runtime:runtime"],
  data = [":notebooks_folder",
          "//python:bark.so",
          "//modules/runtime/tests:xodr_data"]
)

# py_test(
#   name = "notebooks_tests",
#   srcs = ["notebooks_tests.py"],
#   deps = ["//runtime/data/handler:handler",
#             "//runtime/data/handler:json_handler",
#             "//runtime/data/handler:ground_truth",
#             "//runtime/visualization:map",
#             "//runtime/evaluation/preprocessing:preprocessing",
#             "//runtime/visualization:visualize_gaussian2d",            
#             "//runtime/evaluation/metrics:metrics",
#             "//runtime/evaluation/distribution:distribution",
#             "//runtime/data/structure:trajectory",
#             "//runtime/data/sampledata:sample_data_generator",
#             "//runtime/evaluation/evaluator:evaluator"],
#   data = [":notebooks_folder",
#           "//data:real_data"]
# )