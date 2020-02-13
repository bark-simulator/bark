py_library(
    name = "scenario_generation",
    srcs = ["scenario_generation.py",
            "uniform_vehicle_distribution.py",
            "model_json_conversion.py",
            "deterministic.py",
            "deterministic_drone_challenge.py",
            "drone_challenge.py",
            "interaction_dataset_reader.py",
            "interaction_dataset_scenario_generation.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [
        "//modules/runtime/scenario:scenario",
        "@com_github_interaction-dataset_interaction-dataset//:interaction-tools"],
    visibility = ["//visibility:public"],
)