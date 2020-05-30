# Copyright (c) 2020 Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import unittest
import filecmp
import matplotlib.pyplot as plt
from bark.geometry.standard_shapes import *
from modules.runtime.commons.parameters import ParameterServer
from bark.world.opendrive import XodrLaneType
from bark.world.map import MapInterface
from modules.runtime.commons.xodr_parser import XodrParser


def helper_plot(xodr_parser):
  for _, road in xodr_parser.map.GetRoads().items():
    for lane_section in road.lane_sections:
      for _, lane in lane_section.GetLanes().items():
        if lane.lane_type == XodrLaneType.driving:
          color = "grey"
        elif lane.lane_type == XodrLaneType.sidewalk:
          color = "green"
        elif lane.lane_type == XodrLaneType.border:
          color = "red"
        elif lane.lane_type == XodrLaneType.none:
          color = "blue"
        else:
          continue
        line_np = lane.line.ToArray()

        # print(lane.road_mark)
        plt.text(line_np[-1, 0], line_np[-1, 1],
                 'outer_{i}_{j}'.format(i=lane.lane_id, j=lane.lane_position))

        plt.plot(
            line_np[:, 0],
            line_np[:, 1],
            color=color,
            alpha=1.0)

  plt.axis("equal")
  plt.show()


class ImporterTests(unittest.TestCase):
  def test_map_highway(self):
    xodr_parser = XodrParser(
      "modules/runtime/tests/data/city_highway_straight.xodr")

    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    # helper_plot(xodr_parser)


  def test_map_DR_DEU_Merging_MT_v01_shifted(self):
    xodr_parser = XodrParser(
      "modules/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr")

    map_interface = MapInterface()
    map_interface.SetOpenDriveMap(xodr_parser.map)
    # helper_plot(xodr_parser)


if __name__ == '__main__':
  unittest.main()
