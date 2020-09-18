cc_library(
    name = "ad_rss",
    srcs = glob([
        "ad_rss/generated/src/**/*.cpp",
        "ad_rss/impl/src/**/*.cpp",
        "ad_rss/impl/src/**/*.hpp",
    ]),
    hdrs = glob([
        "ad_rss/generated/include/**/*.hpp",
        "ad_rss/impl/include/**/*.hpp",
    ]),
    includes = [
        "ad_rss/generated/include",
        "ad_rss/impl/include",
    ],
    visibility = ["//visibility:public"],
    deps = ["@map_support//:ad_physics", "@boost//:geometry"],
)

# cc_test(
#     name = "ad_rss_test",
#     srcs = glob([
#         "ad_rss/impl/test/**/*.cpp",
#         "ad_rss/impl/test/**/*.hpp",
#     ]),
#     includes=["ad_rss/impl/test/test_support"],
#     deps = [
#         ":ad_rss",
#         "@gtest//:gtest_main",
#     ],
# )

cc_library(
    name = "ad_rss_map_integration",
    srcs = glob([
        "ad_rss_map_integration/generated/src/**/*.cpp",
        "ad_rss_map_integration/impl/src/**/*.cpp",
        "ad_rss_map_integration/impl/src/**/*.hpp",
    ]),
    hdrs = glob([
        "ad_rss_map_integration/generated/include/**/*.hpp",
        "ad_rss_map_integration/impl/include/**/*.hpp",
    ]),
    includes = [
        "ad_rss_map_integration/generated/include",
        "ad_rss_map_integration/impl/include",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":ad_rss",
        "@map_support//:ad_map_access",
        "@map_support//:ad_physics",
    ],
)

# cc_test(
#     name = "ad_rss_map_integration_test",
#     srcs = glob([
#         "ad_rss_map_integration/impl/tests/**/*.cpp",
#         "ad_rss_map_integration/impl/tests/**/*.hpp",
#     ]),
#     includes=["ad_rss_map_integration/impl/tests/"],
#     deps = [
#         ":ad_rss_map_integration",
#         "@gtest//:gtest_main",
#     ],
# )
