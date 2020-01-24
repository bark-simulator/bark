py_library(
    name = "property_readers",
    srcs = [
     "property_readers.py",
     "property_readers_interfaces.py",
     "behavior_model_property_readers.py",
     "agent_state_property_readers.py"
     "model_conversion.py"
     ],
    data = ['//python:bark.so'],
    imports = ['../../../python'],
    deps = ["//modules/runtime/scenario:scenario",],
    visibility = ["//visibility:public"],
)