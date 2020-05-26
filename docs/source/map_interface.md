MapInterface
================================

The `MapInterface` class implements all map related features for BARK.
It stores the raw [OpenDrive](http://www.opendrive.org/download.html) map, has a `RoadGraph` for routing and provides convenient and easy-to-use classes for the agents &dash; the `RoadCorridor` and the `LaneCorridor`.

The `MapInterface` class is implemented as follows:

```cpp
class MapInterface {
 public:
    ...
 private:
  OpenDriveMapPtr open_drive_map_;
  RoadgraphPtr roadgraph_;
  rtree_lane rtree_lane_;
  std::pair<Point2d, Point2d> bounding_box_;
  std::map<std::size_t, RoadCorridorPtr> road_corridors_;
}
```

Additionally, the `MapInterface` also has an lane r-tree for more performant lane searching.



The `OpenDriveMap` class implements the specifications provided by the [OpenDRIVE 1.4 Format](http://www.opendrive.org/download.html).
This allows an easy parsing and integration of maps available in the OpenDrive format.
However, for a better usability we encapsulate this specification using a `RoadGraph`, `RoadCorridors` and `LaneCorridors`.

The basic structure of the `OpenDriveMap` map class:

```cpp
class OpenDriveMap {
 public:
  OpenDriveMap() : roads_(), lanes_(), junctions_() {}
  ~OpenDriveMap() {}
  ...
 private:
  XodrRoads roads_;
  XodrLanes lanes_;
  Junctions junctions_;
}
```

## RoadGraph

The `RoadGraph` contains all roads and lanes and their physical location in a graph structure.
This enables easy routing functionality for an agent in BARK.
The physical locations of the start and goal are sufficient in order to obtain a sequence of lanes or roads for the agent.

However, in order to store this information more efficiently and to increase the usability, we additionally use `RoadCorridors` and `LaneCorridors`.


## RoadCorridor

A `RoadCorridor` is composed out of a sequential sequence of roads that an agent in BARK can follow.
It contains all the OpenDrive information as well as further information in form of the `LaneCorridor`.
The `RoadCorridor` provides an easy-to-use interface by providing functions that tell an agent the current lane and what it left or right lanes are.

The basic structure of the `RoadCorridor` map class:

```cpp
struct RoadCorridor {
  ...
  Roads roads_;
  Polygon road_polygon_;
  std::vector<LaneCorridorPtr> unique_lane_corridors_;
  std::vector<XodrRoadId> road_ids_;
  std::map<LaneId, LaneCorridorPtr> lane_corridors_;
}
```

## LaneCorridor

A `LaneCorridor` are continuously, sequentially concatenated lanes.
It provides many utility functions, such as the distance to the end of the `LaneCorridor`, the merged lane polygon, the boundaries and more.

```cpp
struct LaneCorridor {
  ...
  std::map<float, LanePtr> lanes_;  // s_end, LanePtr
  Line center_line_;
  Polygon merged_polygon_;
  Line left_boundary_;
  Line right_boundary_;
}
```