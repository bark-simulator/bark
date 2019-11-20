filegroup(
    name="carla_python_lib",
    srcs=select({
      "@bazel_tools//src/conditions:linux_x86_64": glob(["carla-*linux-x86_64.egg"]),
      "@bazel_tools//src/conditions:windows": glob(["carla-*win-amd64.egg"])
      }),
    visibility = ["//visibility:public"],
)