cc_library(
    name = "ad_physics",
    srcs = glob(["ad_physics/**/*.cpp"]),
    hdrs = glob(["ad_physics/**/*.hpp"]),
    includes = [
        "ad_physics/generated/include",
        "ad_physics/impl/include",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "ad_map_opendrive_reader",
    srcs = glob(["ad_map_opendrive_reader/src/**/*.cpp"]),
    hdrs = glob(["ad_map_opendrive_reader/include/**/*.h*"]),
    includes = ["ad_map_opendrive_reader/include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "ad_map_access_test_support",
    srcs = glob(["ad_map_access_test_support/**/*.cpp"]),
    hdrs = glob(["ad_map_access_test_support/**/*.hpp"]),
    includes = [
        "ad_map_access_test_support/generated/include",
        "ad_map_access_test_support/impl/include",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "ad_map_access",
    srcs = glob([
        "ad_map_access/generated/src/**/*.cpp",
        "ad_map_access/impl/src/**/*.cpp",
        "ad_map_access/impl/src/**/*.hpp",
    ]),
    hdrs = glob([
        "ad_map_access/generated/include/**/*.hpp",
        "ad_map_access/impl/include/**/*.hpp",
    ]),
    includes = [
        "ad_map_access/generated/include",
        "ad_map_access/impl/include",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    deps = [
        ":ad_map_opendrive_reader",
        ":ad_physics",
    ],
)
