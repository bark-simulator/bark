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
  commit="fac5d85c96867be336a4500d45d0277abe48465c",
  remote = "https://github.com/bark-simulator/planner-mcts"
)
load("@planner_uct//util:deps.bzl", "planner_uct_rules_dependencies")
planner_uct_rules_dependencies()
# --------------------------------------------------

# -------- Benchmark Database -----------------------
git_repository(
  name = "benchmark_database",
  commit="fd185eb87d5e9b256c86fb64bd722dc80b209462",
  remote = "https://github.com/bark-simulator/benchmark-database"
)

load("@benchmark_database//util:deps.bzl", "benchmark_database_dependencies")
load("@benchmark_database//load:load.bzl", "benchmark_database_release")
benchmark_database_dependencies()
benchmark_database_release()
# --------------------------------------------------


# ------------------- LTL RuleMonitor --------------
load("@rule_monitor_project//util:deps.bzl", "rule_monitor_dependencies")
rule_monitor_dependencies()
# --------------------------------------------------

# ------------------- BARK-ML ----------------------
git_repository(
  name = "bark_ml",
  commit="f1f392ab46bd1e5ea8db5958113526c173814a0e",
  remote = "https://github.com/bark-simulator/bark-ml"
)
# --------------------------------------------------

# git_repository(
#  name = "interaction_dataset_fortiss_internal",
#  commit = "9ace5fde9260c20736b0463026e0f407b7d395ba",
#  remote = "https://git.fortiss.org/autosim/interaction_dataset"
# )

