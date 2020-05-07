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
