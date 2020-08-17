py_library(
    name = "config_readers",
    srcs = [
     "__init__.py",
     "config_readers_interfaces.py",
     "agent_state_geometry_config_readers.py",
     "behavior_model_config_readers.py",
     "execution_model_config_readers.py",
     "dynamic_model_config_readers.py",
     "controlled_agents_config_readers.py",
     "goal_definition_config_readers.py",
     ],
    data = ['//bark:generate_core'],
    imports = [],
    deps = [
        "//bark/runtime/scenario:scenario",
        "//bark/runtime/scenario/interaction_dataset_processing:interaction_dataset_processing"
        ],
    visibility = ["//visibility:public"],
)