workspace(name = "bark_project")


load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file = "@//tools:gtest.BUILD",
    strip_prefix = "googletest-release-1.7.0",
)


http_archive(
    name = "com_google_protobuf",
    sha256 = "cef7f1b5a7c5fba672bec2a319246e8feba471f04dcebfe362d55930ee7c1c30",
    strip_prefix = "protobuf-3.5.0",
    urls = ["https://github.com/google/protobuf/archive/v3.5.0.zip"],
)

http_archive(
    name = "pybind11",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
    build_file = "@//tools/pybind11:pybind.BUILD",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    branch = "master",
    remote = "https://github.com/nelhage/rules_boost",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

# External dependency: Eigen; has no Bazel build.
http_archive(
    name = "com_github_eigen_eigen",
    build_file = "@//tools/eigen:eigen.BUILD",
    sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = [
        "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",
    ],
)

# External dependency: Google Flags; has Bazel build already.
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ],
)

# External dependency: Google Log; has Bazel build already.
http_archive(
    name = "com_github_google_glog",
    sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
    strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
    urls = [
        "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
    ],
)

#git_repository(
#    name = "com_google_ceres_solver",
#    commit = "d7f428e5c7907c1dfb2ff34bc543d17a838920a7",
#    remote = "https://github.com/ceres-solver/ceres-solver",
#)

http_archive(
    name = "com_google_ceres_solver", 
    strip_prefix = "ceres-solver-1.14.0", 
    sha256 = "1296330fcf1e09e6c2f926301916f64d4a4c5c0ff12d460a9bc5d4c48411518f",
    build_file = "@//tools/ceres:ceres.BUILD",
    urls = ["https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz"],
)

new_local_repository(
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

local_repository(
  name = "web",
  path = "./web",
)

# ------ Planner UCT --------------
git_repository(
  name = "planner_uct",
  branch="master",
  remote = "https://github.com/bark-simulator/planner-mcts"
)
load("@planner_uct//util:deps.bzl", "planner_uct_rules_dependencies")
planner_uct_rules_dependencies()
# ---------------------------------

# ------ Planner BARK-ML --------------
git_repository(
  name = "bark_ml",
  branch="master",
  remote = "https://github.com/bark-simulator/bark-ml"
)

load("@bark_ml//utils:dependencies.bzl", "load_bark")
load_bark()
# ---------------------------------


# -------- Benchmark Database -----------------------
git_repository(
  name = "benchmark_database",
  branch="master",
  remote = "https://github.com/bark-simulator/benchmark-database"
)

load("@benchmark_database//util:deps.bzl", "benchmark_database_dependencies")
load("@benchmark_database//load:load.bzl", "benchmark_database_release")
benchmark_database_dependencies()
benchmark_database_release()
# --------------------------------------------------