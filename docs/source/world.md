World
================================
The BARK-world models a simultaneous-move game where all agents generate a behavior given a cloned world state at a specific time. All agents are moved according to their specified behavior models for a fixed time period `delta_time` and the validity of the resulting world state is checked. In principle, the `World` class is defined as 

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
  .. cpp:function:: void Step(float delta_time)
  The `Step` function loops through all agents, creates a cloned world and then calls the `Move` function of each agent.
  The Move function in combination with the Step function integrates the main functionality of BARK.
```

```eval_rst
  .. cpp:function:: inline World *World::Clone() const
  
  Returns a cloned world of the current world.
```

## ObservedWorld
The `ObservedWorld` is derived from the (protected) `World`. Furthermore, it offers additional functionality in order to increase its usability in the planning modules. 
In the future, perturbations and other effects could be modeled. For example, the ego agent only having a partial world-view.

```cpp
class ObservedWorld : protected World {
 public:
    ObservedWorld(const World& world, const AgentId& ego_agent_id) :
      World(world),
      ego_agent_id_(ego_agent_id) {}
    ~ObservedWorld() {}
    double GetWorldTime() const { return World::GetWorldTime(); }
    const LocalMap& get_local_map() const {
      return *World::GetAgent(ego_agent_id_)->get_local_map();
    }
    AgentPtr GetEgoAgent() const {
      return World::GetAgent(ego_agent_id_);
    }
    MapInterfacePtr GetMap() const { return World::GetMap(); }
    State CurrentEgoState() const {
      return World::GetAgents()[ego_agent_id_]->GetCurrentState();
    }
    Point2d get_ego_point() const {
      State ego_state = CurrentEgoState();
      return Point2d(ego_state(X_POSITION), ego_state(Y_POSITION));
    }

 private:
    AgentId ego_agent_id_;
};
```

## Agents & Objects
All objects are static with a fixed state and 2D geometry. An agent is a dynamic object functioning as a container for the behavioral-, execution- and dynamic-model as described [here](agent_components.md). The agent class provides a `Move` function taking care of the necessary calls to generate a behavior and to check its feasibility. The agent tracks its movements in the `state_action_history_`. The goal of an agent is initially specified during instantiation using the `goal_lane_id`. Moreover, the `World` class manages all objects and agents based on IDs.

```cpp
	class Agent {
	public:
		 void Move(const float &dt, const ObservedWorld &observed_world);
	private:
		BehaviorModel behavior_model_;
		ExecutionModel execution_model_;
		DynamicModel	dynamic_model_;
	
		StateActionHistory state_action_history_;	
	}
```


```eval_rst
  .. cpp:function:: void Move(const float &dt, const ObservedWorld &observed_world)
  
  The function calls the above-described models sequentially and updates the `state_action_history_`.
```



## OpenDriveMap
The `OpenDriveMap` class implements the specifications provided by the [OpenDRIVE 1.4 Format](http://www.opendrive.org/download.html). This allows an easy parsing and integration of maps available in the OpenDrive format.
Furthermore, it is additionally used as a data-container to store the map in memory.


## MapInterface
Provides an interface to the `OpenDriveMap` in order to speed up the search and it additionally also implements a routing functionality.
Therefore, it utilizes an `R-Tree` in order to find the nearest map objects, such as lanes. Furthermore, all lanes are being saved and connected in a graph.

```cpp
class MapInterface {
	public:
		bool interface_from_opendrive(const OpenDriveMapPtr& open_drive_map);
		bool FindNearestXodrLanes(const modules::geometry::Point2d& point,
                                       const unsigned& num_lanes,
                                       std::vector<opendrive::XodrLanePtr>& lanes);
		std::pair< std::vector<XodrLanePtr>, std::vector<XodrLanePtr> >  
			ComputeXodrLaneBoundariesHorizon(const XodrLaneId& startid, const XodrLaneId& goalid);
		
	private:
	       OpenDriveMapPtr open_drive_map_;
	  	RoadgraphPtr roadgraph_;
	  	rtree_lane rtree_lane_;
}
```

```eval_rst
  .. cpp:function:: std::pair< std::vector<XodrLanePtr>, std::vector<XodrLanePtr> > ComputeXodrLaneBoundariesHorizon(const XodrLaneId& startid, const XodrLaneId& goalid)
  
  Generates a route using boost-graph (the XodrLaneGraph) and returns the left-, right-boundary as well as the centerline.
```


```eval_rst
  .. cpp:function:: bool FindNearestXodrLanes(const modules::geometry::Point2d& point, const unsigned& num_lanes, std::vector<opendrive::XodrLanePtr>& lanes)
  
  A function that returns the nearest lanes for a given point.
```


Based on the Roadgraph-interface, the LocalMap searches a valid route for an agent given its `goal_lane_id`. It then and concatenates the lane geometry to 2D lines which can easily be used in the behavior generation models.

```cpp
class LocalMap {
	public:
		bool generate(Point2d point,
			      XodrLaneId goal_lane_id,
			      double horizon = numeric_double_limits::max());

		Line get_inner_line() const { return inner_line_; }
		Line get_outer_line() const { return outer_line_; }
		Line get_center_line() const { return center_line_; }
	private:
		MapInterfacePtr map_interface_;

}
```
