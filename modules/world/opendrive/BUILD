cc_library(
    name = "opendrive",
    srcs = [
        "odrSpiral.cpp",
        "plan_view.cpp",
        "lane.cpp",
        "road.cpp",
        "lane_section.cpp",
    ],
    hdrs = [
        "lane_section.hpp",
        "road.hpp",
        "lane.hpp",
        "opendrive.hpp",
        "odrSpiral.hpp",        
        "commons.hpp",       
        "plan_view.hpp",        
        "junction.hpp",    
        "helpers.hpp"  
    ],
    deps = [
        "//modules/geometry",
        "@boost//:geometry",
        "//modules/models/dynamic:dynamic"
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name="include",
    hdrs=glob(["**/*.hpp"]),
    visibility = ["//visibility:public"],
)