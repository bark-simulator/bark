# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import os
import unittest
import time
import math
import filecmp
import matplotlib.pyplot as plt
from bark.world import World
from bark.geometry import ComputeCenterLine
from bark.world.map import MapInterface
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np

# Name and Output Directory
# CHANGE THIS #
map_name = "20200403_DR_CHN_Merging_ZS_partial_v02"
output_dir = "/tmp/" + map_name

# Map Definition
xodr_parser = XodrParser("modules/runtime/tests/data/" + map_name + ".xodr")

if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# World Definition
params = ParameterServer()
world = World(params)

map_interface = MapInterface()
map_interface.SetOpenDriveMap(xodr_parser.map)
world.SetMap(map_interface)

open_drive_map = world.map.GetOpenDriveMap()

viewer = MPViewer(params=params,
                  use_world_bounds=True)
viewer.drawWorld(world)
viewer.saveFig(output_dir + "/" + "world_plain.png")

color_triplet_gray = (0.7,0.7,0.7)

# Open Drive Elements (XodrRoads, XodrLane Sections, XodrLanes)
for idx_r, road in open_drive_map.GetRoads().items():
  viewer.drawWorld(world)
  viewer.drawXodrRoad(road)
  viewer.saveFig(output_dir + "/" + "open_drive_map_road_" + str(idx_r) + ".png")
  viewer.show(block=False)
  viewer.clear()

for idx_r, road in open_drive_map.GetRoads().items():
  for idx_ls, lane_section in enumerate(road.lane_sections):
    viewer.drawWorld(world)
    viewer.drawXodrRoad(road, color_triplet_gray)
    viewer.drawXodrLaneSection(lane_section)
    viewer.saveFig(output_dir + "/" + "open_drive_map_road_" + str(idx_r) + "_lane_section" + str(idx_ls) + ".png")
    viewer.show()
    viewer.clear()

for idx_r, road in open_drive_map.GetRoads().items():
  for idx_ls, lane_section in enumerate(road.lane_sections):
    for idx_l, lane in lane_section.GetLanes().items():
      viewer.drawWorld(world)
      viewer.drawXodrRoad(road, color_triplet_gray)
      viewer.drawXodrLaneSection(lane_section, color_triplet_gray)
      viewer.drawXodrLane(lane)
      viewer.saveFig(output_dir + "/" + "open_drive_map_road_" + str(idx_r) + "_lane_section" + str(idx_ls) + "_lane" + str(idx_l) + ".png")
      viewer.show()
      viewer.clear()

# XodrLanes of Roadgraph
roadgraph = map_interface.GetRoadgraph()
roadgraph.PrintGraph(output_dir + "/" + map_name)
lane_ids = roadgraph.GetAllLaneids()

for lane_id in lane_ids:
  lane_polygon = roadgraph.GetLanePolygonForLaneId(lane_id)
  # plot plan_view
  road_id = roadgraph.GetRoadForLaneId(lane_id)
  road = map_interface.GetOpenDriveMap().GetRoad(road_id)
  plan_view_reference = road.plan_view.GetReferenceLine()
  # plot polygon with center line
  viewer.drawWorld(world)
  color = list(np.random.choice(range(256), size=3)/256)
  viewer.drawPolygon2d(lane_polygon, color, 1.0)
  viewer.drawLine2d(plan_view_reference, color="red")
  viewer.saveFig(output_dir + "/" + "roadgraph_laneid_" + str(lane_id) + ".png")
  viewer.show()
  viewer.clear()


#for rc in all_corridors:
#    viewer.drawDrivingCorridor(rc)
#    viewer.saveFig(output_dir + "/" + "test.png")

#
#all_corridors = map_interface.get_all_corridors()
# c = all_corridors[10]
# right_adj_corridors = map_interface.get_adjacent_corridors_same_direction(c, [151, 168, 0.0])
# assert(len(right_adj_corridors) == 2)

# right_adj_corridors = map_interface.get_adjacent_corridors_same_direction(c, [169, 169, 0.0])
# assert(len(right_adj_corridors) == 1)