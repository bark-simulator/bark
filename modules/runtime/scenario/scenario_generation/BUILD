py_library(
    name = "scenario_generation",
    srcs = ["scenario_generation.py",
            "uniform_vehicle_distribution.py",
            "model_json_conversion.py",
            "deterministic.py",
            "deterministic_drone_challenge.py",
            "drone_challenge.py"],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = [
        "//modules/runtime/scenario:scenario",
        "//modules/runtime/scenario/scenario_generation/property_readers:property_readers"
    ],
    visibility = ["//visibility:public"],
)