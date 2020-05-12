cc_library(
    name = "base_rule_based",
    srcs = [
        "base_rule_based.cpp"
    ],
    hdrs = [
        "base_rule_based.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/idm:base_idm",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet",
    ],
    visibility = ["//visibility:public"]
)

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
        "//modules/models/behavior/idm:base_idm",
        "//modules/models/behavior/rule_based:base_rule_based",
        "//modules/models/dynamic:dynamic",
        "//modules/commons/transformation:frenet"
    ],
    visibility = ["//visibility:public"]
)

cc_library(
    name = "simple_behavior",
    srcs = [
      "simple_behavior.cpp"
    ],
    hdrs = [
      "simple_behavior.hpp"
    ],
    deps = [
        "//modules/commons:commons",
        "//modules/world:world",
        "//modules/models/behavior/idm:base_idm",
        "//modules/models/behavior/rule_based:base_rule_based",
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
        "//modules/models/behavior/idm:base_idm",
        "//modules/models/behavior/constant_velocity:constant_velocity",
        "//modules/models/behavior/rule_based:base_rule_based",
        "//modules/models/behavior/rule_based:simple_behavior",
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
        "//modules/models/behavior/rule_based:base_rule_based",
        "//modules/models/behavior/rule_based:simple_behavior",
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