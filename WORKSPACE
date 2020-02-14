workspace(name = "bark_project")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")


# ---- BARK Internal Dependencies ----------------
load("@bark_project//tools:deps.bzl", "bark_dependencies")
bark_dependencies()


load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()
# -------------------------------------------------



# ------ Planner UCT ------------------------------
git_repository(
  name = "planner_uct",
  branch="master",
  remote = "https://github.com/bark-simulator/planner-mcts"
)
load("@planner_uct//util:deps.bzl", "planner_uct_rules_dependencies")
planner_uct_rules_dependencies()
# --------------------------------------------------



# ------ Planner BARK-ML ---------------------------
git_repository(
  name = "bark_ml",
  branch="master",
  remote = "https://github.com/bark-simulator/bark-ml"
)

load("@bark_ml//utils:dependencies.bzl", "load_bark")
load_bark()
# --------------------------------------------------



# -------- Benchmark Database -----------------------
git_repository(
  name = "benchmark_database",
  commit="436e665360daac6ac19285b2aab64bb27a2bc02d",
  remote = "https://github.com/bark-simulator/benchmark-database"
)

load("@benchmark_database//util:deps.bzl", "benchmark_database_dependencies")
load("@benchmark_database//load:load.bzl", "benchmark_database_release")
benchmark_database_dependencies()
benchmark_database_release()
# --------------------------------------------------

new_git_repository(
    name = "com_github_interaction-dataset_interaction-dataset",
    build_file = "@//tools/interaction-dataset:interaction-dataset.BUILD",
    commit = "8e53eecfa9cdcb2203517af2f8ed154ad40c2956",
    remote = "https://github.com/interaction-dataset/interaction-dataset.git",
    shallow_since = "1568028656 +0200",
)

#git_repository(
#  name = "interaction_dataset_fortiss_internal",
#  branch = "master",
#  remote = "https://git.fortiss.org/autosim/interaction_dataset"
#)

