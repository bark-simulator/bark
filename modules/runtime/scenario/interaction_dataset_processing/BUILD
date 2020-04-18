# Flexible command line build configuration of dataset usage
# if you have access to interaction_dataset_fortiss_internal:
# bazel build //modules/runtime/scenario/dataset:dataset_decomposer --define interaction_dataset_present=true

config_setting(
    name = "_interaction_dataset_present",
    define_values = {"interaction_dataset_present": "true"},
)

py_library(
    name = "interaction_dataset_processing",
    srcs = ["dataset_decomposer.py",
            "interaction_dataset_reader.py",
            ],
    data = [
        "//python:bark.so",
        ] + 
    select({":_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),
    imports = ["../../../python/"],
    deps = [
        "//modules/runtime/viewer:matplotlib_viewer",
        "//modules/runtime:runtime",
        "//modules/runtime/commons:model_json_conversion",
        "@com_github_interaction_dataset_interaction_dataset//:interaction-tools",
    ],
    visibility = ["//visibility:public"],
)
