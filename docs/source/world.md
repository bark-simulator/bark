World
================================
The BARK-world models a simultaneous-move game where all agents generate a behavior given a cloned world state at a specific time. All agents are moved according to their specified behavior models for a fixed time period and the validity of the resulting world state is then checked. In principle, the `World` class is defined as 

```cpp
class World {
    public:
	void Step(float delta_time);
    private:
	MapInterface map_interface_;
	AgentMap agents_;
	ObjectMap objects_;
	double world_time_;

}
```

The most important functions in the `World` class are listed below.

```eval_rst
  .. cpp:function:: void Step(const float& delta_time)
```
The `Step` function loops through all agents, creates an observed world and then calls the `BehaviorPlan` and `ExecutionPlan` function of each agent.

```eval_rst
  .. cpp:function:: inline World *World::Clone() const
  
  Returns a cloned world of the current world.
```


## ObservedWorld
The `ObservedWorld` is derived from the (protected) `World`.
It is a modified version of the original world and also has additional functionalities for the ease-of-use.
Perturbations, such as a limitied field of view can be modeled in the observed world as well.


## Agents & Objects
An agent is a dynamic object functioning as a container for the behavioral-, execution- and dynamic-model as described [here](agent_components.md). The agent tracks its movements in the `state_action_history_`. Each agent has a `GoalDefinition` and `RoadCorridor`.


## MapInterface
Provides an interface to the `OpenDriveMap` in order to speed up the search and it additionally also implements a routing functionality.
It utilizes an `R-Tree` in order to find the nearest map objects, such as lanes.

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

### OpenDriveMap
The `OpenDriveMap` class implements the specifications provided by the [OpenDRIVE 1.4 Format](http://www.opendrive.org/download.html). This allows an easy parsing and integration of maps available in the OpenDrive format.
Furthermore, it is additionally used as a data-container to store the map in memory.


### RoadGraph
The roadgraph contains all map information and can be queried to obtain routing specific queries. 

### RoadCorridor
A RoadCorridor is a collection of lanes and also further provides LaneCorridors.
Each agent in BARK has a road corridor in which it can drive.

### LaneCorridor
A LaneCorridor is a continuous driving segment that consists of one more multiple lanes.
