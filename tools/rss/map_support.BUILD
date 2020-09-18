cc_library(
    name = "ad_physics",
    srcs = glob([
        "ad_physics/generated/src/**/*.cpp",
        "ad_physics/impl/src/**/*.cpp",
    ]),
    hdrs = glob([
        "ad_physics/generated/include/**/*.hpp",
        "ad_physics/impl/include/**/*.hpp",
    ]),
    includes = [
        "ad_physics/generated/include",
        "ad_physics/impl/include",
    ],
    visibility = ["//visibility:public"],
    deps = ["@spdlog"],
)

# cc_test(
#     name = "ad_physics_test",
#     srcs = glob(["ad_physics/impl/test/**/*.cpp"]),
#     deps = [
#         "@gtest//:gtest_main",
#         "@map//:ad_physics",
#     ],
# )

cc_library(
    name = "ad_map_opendrive_reader",
    srcs = glob(["ad_map_opendrive_reader/src/**/*.cpp"]),
    hdrs = glob(["ad_map_opendrive_reader/include/**/*.h*"]),
    includes = ["ad_map_opendrive_reader/include"],
    visibility = ["//visibility:public"],
    deps = [
        "@proj",
        "@pugixml",
        "@boost//:program_options",
        "@boost//:array",
        "@boost//:math",
        "@boost//:geometry",
        "@boost//:algorithm",
        "@boost//:filesystem",
    ],
)

cc_library(
    name = "ad_map_access_test_support",
    srcs = glob(["ad_map_access_test_support/**/*.cpp"]),
    hdrs = glob(["ad_map_access_test_support/**/*.hpp"]),
    includes = [
        "ad_map_access_test_support/generated/include",
        "ad_map_access_test_support/impl/include",
    ],
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
    visibility = ["//visibility:public"],
    deps = [
        "@proj",
        ":ad_map_opendrive_reader",
        ":ad_physics",
        "@spdlog",
    ],
)

# There are some error inside their test codes, e.g.
# error: '...' is private within this context

# cc_test(
#     name = "ad_map_access_test",
#     srcs = glob(["ad_map_access/impl/tests/**/*.hpp","ad_map_access/impl/tests/**/*.cpp"]),
#     includes = [
#         # "ad_map_access/generated/include",
#         "ad_map_access/impl/include",
#     ],
#     deps = [
#         ":ad_map_access",
#         ":ad_map_access_test_support",
#         "@gtest//:gtest_main",
#     ],
# )
