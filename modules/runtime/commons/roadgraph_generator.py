# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.world.map import *

class RoadgraphGenerator:
    def __init__(self, roadgraph):
        self.roadgraph = roadgraph

    def get_lane_id_by_pos(self, lane_section, pos):
        for lane_id, lane in lane_section.get_lanes().items():
            if lane.lane_position == pos:
                return lane_id
        return None

    def generate(self, road_map):
        # add vertices
        for road_id, road in road_map.get_roads().items():
            # there could be mult. lane_sections
            for _, lane_section in enumerate(road.lane_sections):
                for lane_id, lane in lane_section.get_lanes().items():
                    self.roadgraph.add_lane(road_id, lane)

        # add successors,predecessors
        for road_id, road in road_map.get_roads().items():
            successor_road_id = road.link.successor.id # this is the position!!!!!! (-4, 4)
            predecessor_road_id = road.link.predecessor.id # this is the position!!!!!! (-4, 4)
            if successor_road_id > 1000:
                continue
            successor_road = road_map.get_roads()[successor_road_id]
            try:
                predecessor_road = road_map.get_roads()[predecessor_road_id]
                # from last element in lane sections
                predecessor_lane_section = predecessor_road.lane_sections[-1]
            except:
                print("Road has no predeseccor road.")

            successor_lane_section = successor_road.lane_sections[0]

            # TODO (@hart): there could be mult. lane_sections
            for _, lane_section in enumerate(road.lane_sections):
                for lane_id, lane in lane_section.get_lanes().items():
                    # add successor edge
                    successor_lane_position = lane.link.successor.id
                    successor_lane_id = self.get_lane_id_by_pos(successor_lane_section, successor_lane_position)
                    if successor_lane_id is not None:
                        self.roadgraph.add_successor(lane_id, successor_lane_id)

                    # does not always have predecessor
                    try:
                        predecessor_lane_position = lane.link.predecessor.id
                        # search for predecessor_lane_position in previos lane section
                        predecessor_lane_id = self.get_lane_id_by_pos(predecessor_lane_section, predecessor_lane_position)
                        # if found add; convert predecessor to successor
                        if predecessor_lane_id is not None:
                            self.roadgraph.add_successor(predecessor_lane_id, lane_id)
                    except:
                        print("Road has no predeseccor road.")

        # add neighbor edges
        for road_id, road in road_map.get_roads().items():
            for _, lane_section in enumerate(road.lane_sections):
                for lane_id, lane in lane_section.get_lanes().items():
                    if lane.lane_position is not 0:
                        inner_lane_pos = lane.lane_position - 1 if lane.lane_position > 0 else lane.lane_position + 1
                        inner_lane_id = self.get_lane_id_by_pos(lane_section, inner_lane_pos)
                        if inner_lane_id is not None:
                            self.roadgraph.add_inner_neighbor(inner_lane_id, lane_id)
                            self.roadgraph.add_outer_neighbor(inner_lane_id, lane_id)



        # map.junctions
        for _, junction in road_map.get_junctions().items():
            for _, connection in junction.get_connections().items():
                incoming_road = road_map.get_roads()[connection.incoming_road]
                connecting_road = road_map.get_roads()[connection.connecting_road]

                pre_lane_section = incoming_road.lane_sections[0]
                successor_lane_section = connecting_road.lane_sections[0]
                for lane_link in connection.lane_links:
                    #print(lane_link.from_id, lane_link.to_id) # TODO (@hart): this is actually the lane pos
                    # add successor edge
                    pre_lane_id = self.get_lane_id_by_pos(pre_lane_section, lane_link.from_id)
                    successor_lane_id = self.get_lane_id_by_pos(successor_lane_section, lane_link.to_id)
                    if successor_lane_id is not None and pre_lane_id is not None:
                        self.roadgraph.add_successor(pre_lane_id, successor_lane_id)