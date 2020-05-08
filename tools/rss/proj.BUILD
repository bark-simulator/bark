load("@bark_project//tools:custom_rules.bzl", "header_template")

header_template(
    name = "proj_config",
    extension = ".h",
    template = "src/proj_config.h.in",
)

cc_library(
    name = "proj",
    srcs = glob([
        "src/**/*.hpp",
        "src/**/*!(test)*.cpp",
        "src/**/*.h",
        "src/**/*.c",
    ]),
    hdrs = glob(["include/**/*.hpp"]),
    includes = [
        "include",
        "src",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    deps = [":proj_config"],
)
