workspace(name = "bark_project")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository", "new_git_repository")


# ---- BARK Internal Dependencies ----------------
load("@bark_project//tools:deps.bzl", "bark_dependencies")
bark_dependencies()


load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python")

# -------------------------------------------------

# ------ Planner UCT ------------------------------
#git_repository(
#  name = "planner_uct",
#  commit="5af72cd8a2b0ae0417487e26fc899d2471552aa0",
#  remote = "https://github.com/juloberno/bark_hypothesis_uct"
#)
#load("@planner_uct//util:deps.bzl", "planner_uct_rules_dependencies")
#planner_uct_rules_dependencies()
# --------------------------------------------------

# ------ Planner MIQP ------------------------------
# git_repository(
#     name = "planner_miqp",
#     commit="cea69161e7763af59d76bad87d8b4d6e4d0d1a8c",
#     remote = "https://github.com/bark-simulator/planner-miqp"
# )
# load("@planner_miqp//util:deps.bzl", "planner_miqp_dependencies")
# planner_miqp_dependencies()

# ------ Planner Rules-MCTS ------------------------------
git_repository(
  name = "planner_rules_mcts",
  commit="35b09a857d8c3f1c65e0ed80ee4df1b358e45bf4",
  remote = "https://github.com/bark-simulator/planner-rules-mcts"
)
load("@planner_rules_mcts//util:deps.bzl", "planner_rules_mcts_dependencies")
planner_rules_mcts_dependencies()
# --------------------------------------------------

# -------- Benchmark Database -----------------------
git_repository(
  name = "benchmark_database",
  commit="bdf028c3b11636d5ea327fe30e5bb7091aefbc8a",
  remote = "https://github.com/bark-simulator/benchmark-database"
)

load("@benchmark_database//util:deps.bzl", "benchmark_database_dependencies")
load("@benchmark_database//load:load.bzl", "benchmark_database_release")
benchmark_database_dependencies()
benchmark_database_release()
# --------------------------------------------------

# ------------------- BARK-ML ----------------------
git_repository(
  name = "bark_ml",
  commit="f1f392ab46bd1e5ea8db5958113526c173814a0e",
  remote = "https://github.com/bark-simulator/bark-ml"
)
# --------------------------------------------------

# ------------------- LTL RuleMonitor --------------
load("@rule_monitor_project//util:deps.bzl", "rule_monitor_dependencies")
rule_monitor_dependencies()
# --------------------------------------------------

# ---------------------- RSS -----------------------
load("@com_github_rules_rss//rss:rss.bzl", "rss_dependencies")
rss_dependencies()
# --------------------------------------------------

# -------------------- BARKSCAPE -------------------
load("@barkscape_project//utils:dependencies.bzl", "dependencies")
dependencies()
load("@build_bazel_rules_nodejs//:index.bzl", "yarn_install")
yarn_install(
    # Name this npm so that Bazel Label references look like @npm//package
    name = "npm",
    package_json = "@barkscape_project//barkscape/web:package.json",
    yarn_lock = "@barkscape_project//barkscape/web:yarn.lock",
)
# --------------------------------------------------

# git_repository(
#  name = "interaction_dataset_fortiss_internal",
#  commit = "9ace5fde9260c20736b0463026e0f407b7d395ba",
#  remote = "https://git.fortiss.org/autosim/interaction_dataset"
# )

