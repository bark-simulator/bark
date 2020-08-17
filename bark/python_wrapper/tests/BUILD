package(default_visibility = ["//visibility:public"])

py_test(
  name = "py_pickle_tests",
  srcs = ["py_pickle_tests.py"],
  data = ['//bark:generate_core'],
  deps = ["//bark/runtime/commons:commons"]
)

py_test(
  name = "py_glog_init",
  srcs = ["py_glog_init.py"],
  data = ['//bark:generate_core'],
  imports = ['../../python/']
)

cc_library(
    name="logging_tests",
    hdrs=["logging_tests.hpp"],
    deps=["//bark/commons:commons"],
    visibility = ["//visibility:public"],
)