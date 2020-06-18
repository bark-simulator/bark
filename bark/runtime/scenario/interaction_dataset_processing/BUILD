# Flexible command line build configuration of dataset usage
# if you have access to interaction_dataset_fortiss_internal:
# bazel build //bark/runtime/scenario/dataset:dataset_decomposer --define interaction_dataset_present=true

config_setting(
    name = "_interaction_dataset_present",
    define_values = {"interaction_dataset_present": "true"},
)

py_library(
    name = "interaction_dataset_processing",
    srcs = ["dataset_decomposer.py",
            "interaction_dataset_reader.py",
            "agent_track_info.py",
            "scenario_track_info.py",
            "__init__.py"
            ],
    data = [
        "//bark:generate_core",
        ] + 
    select({":_interaction_dataset_present" : ["@interaction_dataset_fortiss_internal//:maps_tracks"], "//conditions:default": []}),
    imports = ["../../../python/"],
    deps = [
        "//bark/runtime/viewer:matplotlib_viewer",
        "//bark/runtime:runtime",
        "//bark/runtime/commons:commons",
        "@com_github_interaction_dataset_interaction_dataset//:interaction-tools",
    ],
    visibility = ["//visibility:public"],
)
