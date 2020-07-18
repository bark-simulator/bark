# Flexible command line build configuration of the external planners
# Usage:
# 1) e.g. build planners planner_uct and planner_other2 bazel build //bark/python_wrapper/plan:planners --define planner_uct=true --define planner_other2=true
# 2) build no planner (default): bazel build //bark/python_wrapper/plan:planners
#
# Definition of defines in //bark/models/behavior/plan/BUILD


cc_library(
    name="planners",
    hdrs=["plan.hpp"],
    srcs=["plan.cpp"],
    deps= ["//bark/python_wrapper:common",
           "@boost//:geometry"] + 
        select({"//bark/models/behavior/plan:_planner_uct" : ["@planner_uct//bark_mcts/python_wrapper:planner_uct"], "//conditions:default": []}) + 
        select({"//bark/models/behavior/plan:_planner_other1" : ["insert_dependency_other1_here"], "//conditions:default": []}) + 
        select({"//bark/models/behavior/plan:_planner_other2" : ["insert_dependency_other2_here"], "//conditions:default": []}),
    copts=
        select({"//bark/models/behavior/plan:_planner_uct" : ["-D PLANNER_UCT"], "//conditions:default": []}) + 
        select({"//bark/models/behavior/plan:_planner_other1" : ["insert_dependency_other1_here"], "//conditions:default": []}) + 
        select({"//bark/models/behavior/plan:_planner_other2" : ["insert_dependency_other2_here"], "//conditions:default": []}),
    visibility = ["//visibility:public"],
)
