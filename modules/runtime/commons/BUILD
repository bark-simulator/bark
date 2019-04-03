py_library(
    name = "parameters",
    srcs = ["parameters.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [],
    visibility = ["//visibility:public"],
)

py_library(
    name = "commons",
    srcs = ["commons.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [],
    visibility = ["//visibility:public"],
)

py_library(
    name = "xodr_parser",
    srcs = ["xodr_parser.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [":roadgraph_generator"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "roadgraph_generator",
    srcs = ["roadgraph_generator.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [],
    visibility = ["//visibility:public"],
)