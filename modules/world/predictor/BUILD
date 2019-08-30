cc_library(
    name = "predictor",
    srcs = [
        "prediction.cpp"
    ],
    hdrs = [
        "commons.hpp",
        "prediction.hpp",
        "agent_prediction.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/models/behavior:behavior",
        "//modules/world:world",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/behavior/pure_pursuit:pure_pursuit"
    ],
    visibility = ["//visibility:public"],
)
