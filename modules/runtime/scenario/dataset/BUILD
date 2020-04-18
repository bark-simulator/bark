# Flexible command line build configuration of dataset usage
# if you have access to interaction_dataset_fortiss_internal:
# bazel build //modules/runtime/scenario/dataset:dataset_decomposer --define interaction_dataset_present=true

config_setting(
    name = "_interaction_dataset_present",
    define_values = {"interaction_dataset_present": "true"},
)

py_library(
    name = "dataset_decomposer",
    srcs = ["dataset_decomposer.py"],
    data = [
        "//python:bark.so",
        ] + 
    select({":_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime/viewer:matplotlib_viewer",
        "//modules/runtime:runtime",
    ],
    visibility = ["//visibility:public"],
)
