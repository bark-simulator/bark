cc_library(
    name = "rule_based",
    srcs = [
        "rule_based.cpp"
    ],
    hdrs = [
        "rule_based.hpp"
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
        "//modules/models/behavior/rule_based:rule_based",
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
        "//modules/models/behavior/rule_based:rule_based",
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