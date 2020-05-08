load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

def bark_dependencies():
    _maybe(
    git_repository,
    name = "bark_project",
    branch = "master",
    remote = "https://github.com/bark-simulator/bark"
    )

    _maybe(
    git_repository,
    name = "gtest",
    commit = "482ac6ee63429af2aa9c44f4e6427873fb68fb1f",
    remote = "https://github.com/google/googletest"
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
    new_git_repository,
    name = "com_github_interaction_dataset_interaction_dataset",
    build_file = "@bark_project//tools/interaction-dataset:interaction-dataset.BUILD",
    commit = "8e53eecfa9cdcb2203517af2f8ed154ad40c2956",
    remote = "https://github.com/interaction-dataset/interaction-dataset.git",
    shallow_since = "1568028656 +0200",
    )


    # Need Eigen 3.4 (which is under development) for STL-compatible iterators
    _maybe(
    http_archive,
    name = "com_github_eigen_eigen",
    build_file = "@bark_project//tools/eigen:eigen.BUILD",
    sha256 = "4b1120abc5d4a63620a886dcc5d7a7a27bf5b048c8c74ac57521dd27845b1d9f",
    strip_prefix = "eigen-git-mirror-98e54de5e25aefc6b984c168fb3009868a93e217",
    urls = ["https://github.com/eigenteam/eigen-git-mirror/archive/98e54de5e25aefc6b984c168fb3009868a93e217.zip"],
    )

    _maybe(
      git_repository,
      name = "com_github_gflags_gflags",
      commit = "addd749114fab4f24b7ea1e0f2f837584389e52c",
      remote = "https://github.com/gflags/gflags"
    )

    _maybe(
      git_repository,
      name = "com_github_google_glog",
      commit = "195d416e3b1c8dc06980439f6acd3ebd40b6b820",
      remote = "https://github.com/google/glog"
    )

    # -------- ad-rss-lib -----------------------
    # Build steps:
    # ad-rss-lib: https://intel.github.io/ad-rss-lib/BUILDING/index.html
    # map_support: https://ad-map-access.readthedocs.io/en/latest/BUILDING/index.html

    _maybe(
    http_archive,
    name = "ad_rss_lib",
    build_file = "@bark_project//tools/rss:rss.BUILD",
    sha256 = "b77c658434399815f02dd12ecf83e40f8bf015ca29a8407c4361a3b3b0c28eb7",
    strip_prefix = "ad-rss-lib-3.0.0",
    urls = ["https://github.com/intel/ad-rss-lib/archive/v3.0.0.tar.gz"],
    )

    _maybe(
      http_archive,
      name = "spdlog",
      build_file = "@bark_project//tools/rss:spdlog.BUILD",
      sha256 = "b38e0bbef7faac2b82fed550a0c19b0d4e7f6737d5321d4fd8f216b80f8aee8a",
      strip_prefix = "spdlog-1.5.0",
      urls = ["https://github.com/gabime/spdlog/archive/v1.5.0.tar.gz"],
    )

    _maybe(
    http_archive,
    name = "map_support",
    build_file = "@bark_project//tools/rss:map_support.BUILD",
    sha256 = "667a16c029854c51a60ae88f8ef0c24542d7683a78c4f85ec4845c8555d72ac6",
    strip_prefix = "map-2.0.0",
    urls = ["https://github.com/carla-simulator/map/archive/v2.0.0.tar.gz"],
    )

    _maybe(
    http_archive,
    name = "pugixml",
    build_file = "@bark_project//tools/rss:pugixml.BUILD",
    sha256 = "55f399fbb470942410d348584dc953bcaec926415d3462f471ef350f29b5870a",
    strip_prefix = "pugixml-1.10",
    urls = ["https://github.com/zeux/pugixml/releases/download/v1.10/pugixml-1.10.tar.gz"],
    )
    
    _maybe(
    http_archive,
    name = "proj",
    build_file = "@bark_project//tools/rss:proj.BUILD",
    sha256 = "a7026d39c9c80d51565cfc4b33d22631c11e491004e19020b3ff5a0791e1779f",
    strip_prefix = "proj-7.0.1",
    urls = ["https://download.osgeo.org/proj/proj-7.0.1.tar.gz"],
    )
    # -------------------------------------------

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


