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
from bark.geometry import compute_center_line
from bark.world.map import MapInterface
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.commons.xodr_parser import XodrParser
from modules.runtime.viewer.matplotlib_viewer import MPViewer
import numpy as np

# Name and Output Directory
# CHANGE THIS #
map_name = "4way_intersection"
output_dir = "/home/hart/Dokumente/2020/" + map_name

# Map Definition
xodr_parser = XodrParser("modules/runtime/tests/data/" + map_name + ".xodr")

if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# World Definition
params = ParameterServer()
world = World(params)

map_interface = MapInterface()
map_interface.set_open_drive_map(xodr_parser.map)
world.set_map(map_interface)

open_drive_map = world.map.get_open_drive_map()

viewer = MPViewer(params=params,
                  use_world_bounds=True)
viewer.drawWorld(world)
viewer.saveFig(output_dir + "/" + "world_plain.png")

color_triplet_gray = (0.7,0.7,0.7)

# Open Drive Elements (Roads, Lane Sections, Lanes)
for idx_r, road in open_drive_map.get_roads().items():
  viewer.drawWorld(world)
  viewer.drawRoad(road)
  viewer.saveFig(output_dir + "/" + "open_drive_map_road_" + str(idx_r) + ".png")
  viewer.show()
  viewer.clear()

for idx_r, road in open_drive_map.get_roads().items():
  for idx_ls, lane_section in enumerate(road.lane_sections):
    viewer.drawWorld(world)
    viewer.drawRoad(road, color_triplet_gray)
    viewer.drawLaneSection(lane_section)
    viewer.saveFig(output_dir + "/" + "open_drive_map_road_" + str(idx_r) + "_lane_section" + str(idx_ls) + ".png")
    viewer.show()
    viewer.clear()

for idx_r, road in open_drive_map.get_roads().items():
  for idx_ls, lane_section in enumerate(road.lane_sections):
    for idx_l, lane in lane_section.get_lanes().items():
      viewer.drawWorld(world)
      viewer.drawRoad(road, color_triplet_gray)
      viewer.drawLaneSection(lane_section, color_triplet_gray)
      viewer.drawLane(lane)
      viewer.saveFig(output_dir + "/" + "open_drive_map_road_" + str(idx_r) + "_lane_section" + str(idx_ls) + "_lane" + str(idx_l) + ".png")
      viewer.show()
      viewer.clear()

# Lanes of Roadgraph
roadgraph = map_interface.get_roadgraph()
roadgraph.print_graph(output_dir + "/" + map_name)
lane_ids = roadgraph.get_all_laneids ()

for lane_id in lane_ids:
  lane_polygon = roadgraph.get_lane_polygon_by_id(lane_id)
  # plot plan_view
  road_id = roadgraph.get_road_by_lane_id(lane_id)
  road = map_interface.get_open_drive_map().get_road(road_id)
  plan_view_reference = road.plan_view.get_reference_line()
  # plot polygon with center line
  outer, inner = roadgraph.compute_lane_boundaries(lane_id)
  center_line = compute_center_line(outer.line, inner.line)
  viewer.drawWorld(world)
  color = list(np.random.choice(range(256), size=3)/256)
  viewer.drawPolygon2d(lane_polygon, color, 1.0)
  viewer.drawLine2d(center_line)
  viewer.drawLine2d(plan_view_reference, color="red")
  viewer.saveFig(output_dir + "/" + "roadgraph_laneid_" + str(lane_id) + ".png")
  viewer.show(block=False)


#for rc in all_corridors:
#    viewer.drawDrivingCorridor(rc)
#    viewer.saveFig(output_dir + "/" + "test.png")

#map_interface.compute_all_driving_corridors()#
#
#all_corridors = map_interface.get_all_corridors()
# c = all_corridors[10]
# right_adj_corridors = map_interface.get_adjacent_corridors_same_direction(c, [151, 168, 0.0])
# assert(len(right_adj_corridors) == 2)

# right_adj_corridors = map_interface.get_adjacent_corridors_same_direction(c, [169, 169, 0.0])
# assert(len(right_adj_corridors) == 1)