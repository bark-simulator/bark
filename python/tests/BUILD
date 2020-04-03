py_test(
  name = "py_pickle_tests",
  srcs = ["py_pickle_tests.py"],
  data = ['//python:bark.so'],
  imports = ['../../python/'],
  deps = ["//modules/runtime/commons:parameters"]
)

py_test(
  name = "py_glog_init",
  srcs = ["py_glog_init.py"],
  data = ['//python:bark.so'],
  imports = ['../../python/']
)

cc_library(
    name="logging_tests",
    hdrs=["logging_tests.hpp"],
    deps=["//modules/commons:commons"],
    visibility = ["//visibility:public"],
)