# Flexible command line build configuration of the external planners
# Usage:
# 1) e.g. build planners planner_uct and planner_other2 bazel build //bark/models/behavior/plan:planners --define planner_uct=true --define planner_other2=true
# 2) build no planner (default): bazel build //bark/models/behavior/plan:planners

config_setting(
    name = "_planner_uct",
    define_values = {"planner_uct": "true"},
)

config_setting(
    name = "_planner_other1",
    define_values = {"planner_other1": "true"},
)

config_setting(
    name = "_planner_other2",
    define_values = {"planner_other2": "true"},
)


cc_library(
    name="planners",
    deps=
        select({":_planner_uct" : ["@planner_uct//modules/models/behavior:planner_uct_single_agent"], "//conditions:default": []}) + 
        select({":_planner_other1" : ["insert_dependency_other1_here"], "//conditions:default": []}) + 
        select({":_planner_other2" : ["insert_dependency_other2_here"], "//conditions:default": []}),
)

# python planners
config_setting(
    name = "_bark_ml",
    define_values = {"bark_ml": "true"},
)

py_library(
  name="py_planners",
  deps=select({":_bark_ml":
      ["@bark_ml//configurations:bark_agent",
       "@bark_ml//configurations/sac_highway:configuration"],
      "//conditions:default": []
    }),
  visibility = ["//visibility:public"]
)