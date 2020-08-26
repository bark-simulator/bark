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
    strip_prefix = "pybind11-2.5.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.5.0.zip"],
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
      new_git_repository,
      name = "com_github_google_glog",
      commit = "195d416e3b1c8dc06980439f6acd3ebd40b6b820",
      remote = "https://github.com/google/glog",
      build_file_content = """
load("//:bazel/glog.bzl", "glog_library")
glog_library(with_gflags=0)
    """
    )

    # -------- ad-rss-lib -----------------------
    # Build steps:
    # ad-rss-lib: https://intel.github.io/ad-rss-lib/BUILDING/index.html
    # map_support: https://ad-map-access.readthedocs.io/en/latest/BUILDING/index.html

    _maybe(
    http_archive,
    name = "ad_rss_lib",
    build_file = "@bark_project//tools/rss:rss.BUILD",
    sha256 = "a804472ede894f6323fb8de1ba34f2a0d6ff92603a48709a029e3c1ec28fe0ed",
    strip_prefix = "ad-rss-lib-4.0.1",
    urls = ["https://github.com/intel/ad-rss-lib/archive/v4.0.1.tar.gz"],
    )

    _maybe(
      http_archive,
      name = "spdlog",
      build_file = "@bark_project//tools/rss:spdlog.BUILD",
      sha256 = "f0114a4d3c88be9e696762f37a7c379619443ce9d668546c61b21d41affe5b62",
      strip_prefix = "spdlog-1.7.0",
      urls = ["https://github.com/gabime/spdlog/archive/v1.7.0.tar.gz"],
    )

    _maybe(
    http_archive,
    name = "map_support",
    build_file = "@bark_project//tools/rss:map_support.BUILD",
    sha256 = "4613e353ced9d140e489f028a15f1afd9ba0c965feb63c36bbc950f6f30c8ee8",
    strip_prefix = "map-2.2.0",
    urls = ["https://github.com/carla-simulator/map/archive/v2.2.0.tar.gz"],
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
    sha256 = "876151e2279346f6bdbc63bd59790b48733496a957bccd5e51b640fdd26eaa8d",
    strip_prefix = "proj-7.1.0",
    urls = ["https://download.osgeo.org/proj/proj-7.1.0.tar.gz"],
    )
    # -------------------------------------------

    _maybe(
    native.new_local_repository,
    name = "python_linux",
    path = "./bark/python_wrapper/venv/",
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

    _maybe(
    git_repository,
    name = "rule_monitor_project",
    commit = "3bf19a30c72d615af3265c22c391d9bbc6806680",
    remote = "https://github.com/bark-simulator/rule-monitoring.git",
    )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)


