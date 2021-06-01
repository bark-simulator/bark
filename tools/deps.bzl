load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")

def bark_dependencies():
    _maybe(
        git_repository,
        name = "pybind11_bazel",
        commit="c4a29062b77bf42836d995f6ce802f642cffb939",
        remote = "https://github.com/bark-simulator/pybind11_bazel"
    )

    _maybe(
    git_repository,
    name = "com_github_rules_rss",
    commit = "f28d98d75010204db6a96ab1949f8346ba7e3237",
    remote = "https://github.com/bark-simulator/rules_rss"
    )

    _maybe(
    git_repository,
    name = "gtest",
    commit = "703bd9caab50b139428cea1aaff9974ebee5742e",
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
    git_repository,
    name = "com_github_glog_glog",
    commit = "c5dcae830670bfaea9573fa7b700e862833d14ff",
    remote = "https://github.com/google/glog.git"
    )

    _maybe(
      new_git_repository,
      name = "com_github_spline",
      commit = "619c634ef5f6f2df1508c767f979eb4b7bf9c66a",
      remote = "https://github.com/ttk592/spline",
      build_file_content = """
cc_library(
    name = 'spline',
    srcs = [],
    includes = ['.'],
    hdrs = ["src/spline.h"],
    visibility = ['//visibility:public'],
)
    """
    )

    _maybe(
    git_repository,
    name = "rule_monitor_project",
    commit = "43fda8d3d545266e781f76d91e19c35a3fdd6c30",
    remote = "https://github.com/bark-simulator/rule-monitoring.git",
    )

    _maybe(
    git_repository,
    name = "barkscape_project",
    branch = "master",
    remote = "https://github.com/bark-simulator/barkscape.git",
    )

    # _maybe(
    #     native.local_repository,
    #     name = "barkscape_project",
    #     path = "/Users/hart/Development/barkscape",
    # )

def _maybe(repo_rule, name, **kwargs):
    if name not in native.existing_rules():
        repo_rule(name = name, **kwargs)


