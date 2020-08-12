cc_library(
    name = "mobil",
    srcs = [
        "mobil.cpp"
    ],
    hdrs = [
        "mobil.hpp"
    ],
    deps = [
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/rule_based:lane_change_behavior",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet"
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
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/idm:idm_lane_tracking",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet"
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
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/behavior/rule_based:lane_change_behavior",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet"
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
        "//bark/commons:commons",
        "//bark/world:world",
        "//bark/models/behavior/idm:base_idm",
        "//bark/models/behavior/constant_acceleration:constant_acceleration",
        "//bark/models/behavior/rule_based:lane_change_behavior",
        "//bark/models/dynamic:dynamic",
        "//bark/commons/transformation:frenet"
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name="include",
    hdrs=glob(["*.hpp"]),
    visibility = ["//visibility:public"],
)
