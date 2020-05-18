cc_library(
    name = "mobil",
    srcs = [
        "mobil.cpp"
    ],
    hdrs = [
        "mobil.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/rule_based:lane_change_behavior",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet"
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name = "lane_change_behavior",
    srcs = [
      "lane_change_behavior.cpp"
    ],
    hdrs = [
      "lane_change_behavior.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/idm:idm_lane_tracking",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet"
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name = "intersection_behavior",
    srcs = [
      "intersection_behavior.cpp"
    ],
    hdrs = [
      "intersection_behavior.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/behavior/rule_based:lane_change_behavior",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet"
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name = "mobil_behavior",
    srcs = [
      "mobil_behavior.cpp"
    ],
    hdrs = [
      "mobil_behavior.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/idm:base_idm",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/behavior/rule_based:lane_change_behavior",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet"
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)