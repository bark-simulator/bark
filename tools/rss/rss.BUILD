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
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    deps = ["@map//:ad_physics"],
)

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
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    deps = [
        ":ad_rss",
        "@map//:ad_map_access",
        "@map//:ad_physics",
    ],
)
