World
================================

The world in BARK contains all objects of the simulation.
It is modeled as a simultaneous-move game where all agents act at the same time.
Each agent moves according to its defined behavior, execution, and dynamic model.
After all agents have been moved, the overall validity is then checked.

The `World` class is defined as

```cpp
class World {
  public:
    void Step(float delta_time);
  ...
  private:
    MapInterface map_interface_;
    AgentMap agents_;
    ObjectMap objects_;
    double world_time_;
}.
```

The [MapInterface](map_interface.md) contains the map, functionalities for routing, and simplified structures for the agents to plan in.
The `AgentMap` contains all agents of the simulation and the `ObjectMap` all static objects.
Finally, the `World` class also contains the simulation world time `world_time_`.


## Observed World

In each simulation step, an agent in BARK gets passed an `ObservedWorld` that is derived from the current `World`.
The agent then plans in this derived world and returns a trajectory.
The `ObservedWorld` provides additional interfaces and allows to model further features, such as e.g. occlusions.
Besides providing additional functionalities, it also defines and saves the ego agent id `ego_agent_id_`.

```cpp
class ObservedWorld : public World {
 public:
  ObservedWorld(const WorldPtr& world, const AgentId& ego_agent_id) :
    World(world),
    ego_agent_id_(ego_agent_id) {}
  ...
 private:
  AgentId ego_agent_id_;
};
```

## Objects and Agents

In BARK objects are static and can be extended to dynamic agents.
Objects in BARK have a position, a shape, and an ObjectId.
The agent extends this by adding a behavior, execution, and dynamic model as described [here](models.md).
Additionally, the agent also has a  `GoalDefinitionPtr` and `RoadCorridorPtr`.

The agent class is defined as follows:

```cpp
class Agent : public Object {
 public:
  friend class World;

  Agent(const State& initial_state,
        const BehaviorModelPtr& behavior_model_ptr,
        const DynamicModelPtr& dynamic_model_ptr,
        const ExecutionModelPtr& execution_model,
        const geometry::Polygon& shape,
        const commons::ParamsPtr& params,
        const GoalDefinitionPtr& goal_definition = GoalDefinitionPtr(),
        const MapInterfacePtr& map_interface = MapInterfacePtr(),
        const geometry::Model3D& model_3d = geometry::Model3D());
  ...
 private:
  BehaviorModelPtr behavior_model_;
  DynamicModelPtr dynamic_model_;
  ExecutionModelPtr execution_model_;
  RoadCorridorPtr road_corridor_;
  StateActionHistory history_;
  uint32_t max_history_length_;
  GoalDefinitionPtr goal_definition_;
};
```