py_library(
    name = "parameters",
    srcs = ["parameters.py","__init__.py"],
    data = ['//bark:generate_core'],
    deps = [],
    visibility = ["//visibility:public"],
)

py_library(
    name = "commons",
    srcs = ["commons.py","__init__.py","xodr_parser.py","model_json_conversion.py","parameters.py"],
    data = ['//bark:generate_core'],
    #deps = [':xodr_parser',':model_json_conversion',':parameters'],
    visibility = ["//visibility:public"],
)

py_library(
    name = "xodr_parser",
    srcs = ["xodr_parser.py","__init__.py"],
    data = ['//bark:generate_core'],
    deps = [],
    visibility = ["//visibility:public"],
)

py_library(
    name = "model_json_conversion",
    srcs = ["model_json_conversion.py","__init__.py"],
    data = ['//bark:generate_core'],
    deps = [],
    visibility = ["//visibility:public"],
)
