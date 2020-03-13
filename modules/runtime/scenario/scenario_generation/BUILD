py_library(
    name = "scenario_generation",
    srcs = ["scenario_generation.py",
            "uniform_vehicle_distribution.py",
            "configurable_scenario_generation.py",
            "model_json_conversion.py",
            "deterministic.py",
            "config_with_ease.py",
            "deterministic_drone_challenge.py",
            "drone_challenge.py",
            "interaction_dataset_reader.py",
            "interaction_dataset_scenario_generation.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [
        "//modules/runtime/scenario:scenario",
        "@com_github_interaction_dataset_interaction_dataset//:interaction-tools",
        "//modules/runtime/scenario/scenario_generation/config_readers:config_readers"
    ],
    visibility = ["//visibility:public"],
)