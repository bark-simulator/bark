load("@bark_project//tools:custom_rules.bzl", "header_template")

header_template(
    name = "proj_config",
    extension = ".h",
    template = "src/proj_config.h.in",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "proj",
    srcs = glob([
        "src/**/*.cpp",
        "src/**/*.c",
    ]),
    hdrs = glob([
        "include/**/*.hpp",
        "src/**/*.h",
        "src/**/*.hpp",
    ]) + [
        "proj_config.h",
    ],
    includes = [
        ".",
        "include",
        "src",
    ],
    linkopts = [
        "-pthread",
        "-lsqlite3",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":proj_config",
        "@boost//:algorithm",
        "@boost//:array",
        "@boost//:functional",
        "@boost//:geometry",
        "@boost//:lexical_cast",
        "@boost//:math",
        "@boost//:program_options",
        "@gtest",
    ],
)
