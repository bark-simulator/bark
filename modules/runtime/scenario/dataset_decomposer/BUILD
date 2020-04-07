py_binary(
    name = "dataset_decomposer",
    srcs = ["dataset_decomposer.py"],
    data = [
        "//python:bark.so",
        "@interaction_dataset_fortiss_internal//:maps_tracks",
        ],
    imports = ["../python/"],
    deps = [
        "//modules/runtime/scenario/scenario_generation",
        "//modules/runtime/viewer:matplotlib_viewer",
        "//modules/runtime:runtime",
    ],
)