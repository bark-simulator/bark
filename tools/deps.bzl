load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

def bark_dependencies():
    _maybe(
    git_repository,
    name = "bark_project",
    # commit="846c3a736a2606a7aeb067a55b25b9e354bd25bf",
    branch = "master",
    remote = "https://github.com/bark-simulator/bark"
    )

    _maybe(
    http_archive,
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file = "@bark_project//tools:gtest.BUILD",
    strip_prefix = "googletest-release-1.7.0"
    )

    _maybe(
    http_archive,
    name = "pybind11",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
    build_file = "@bark_project//tools/pybind11:pybind.BUILD"
    )

    _maybe(
    git_repository,
    name = "com_github_nelhage_rules_boost",
    branch = "master",
    remote = "https://github.com/nelhage/rules_boost"
    )

    _maybe(
    git_repository,
    name = "bark_ml",
    branch = "master",
    remote = "https://github.com/bark-simulator/bark-ml"
    )

    _maybe(
    new_git_repository,
    name = "com_github_interaction_dataset_interaction_dataset",
    build_file = "@bark_project//tools/interaction-dataset:interaction-dataset.BUILD",
    commit = "8e53eecfa9cdcb2203517af2f8ed154ad40c2956",
    remote = "https://github.com/interaction-dataset/interaction-dataset.git",
    shallow_since = "1568028656 +0200",
    )


    _maybe(
    http_archive,
    name = "com_github_eigen_eigen",
    build_file = "@bark_project//tools/eigen:eigen.BUILD",
    sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = [
        "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",
    ]
    )

    _maybe(
    http_archive,
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ]
    )

    _maybe(
    http_archive,
    name = "com_github_google_glog",
    sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
    strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
    urls = [
        "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
    ]
    )

    _maybe(
    http_archive,
    name = "com_google_ceres_solver", 
    strip_prefix = "ceres-solver-1.14.0", 
    sha256 = "1296330fcf1e09e6c2f926301916f64d4a4c5c0ff12d460a9bc5d4c48411518f",
    build_file = "@bark_project//tools/ceres:ceres.BUILD",
    urls = ["https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz"]
    )

    _maybe(
    native.new_local_repository,
    name = "python_linux",
    path = "./python/venv/",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"], 
    visibility = ["//visibility:public"],
)
    """
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)


