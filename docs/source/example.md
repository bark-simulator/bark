Examples
=========================
In order to get started with BARK, we provide several examples that show the basic functionality. All examples are found within the `/examples`-directory.

## Constant Velocity OpenDrive 8

This example shows how to place a single agent in the [OpenDrive example map "standard crossing"](http://www.opendrive.org/download.html) and how to simulate it with a constant-velocity.
To run this example use the command `bazel run //examples:od8_const_vel_one_agent` (make sure to be in the virtual environment!) which yields:

```eval_rst
.. image:: gifs/open_drive8_near.gif
   :scale: 64 %
   :align: left
.. image:: gifs/open_drive8_far.gif
   :scale: 64 %
   :align: left
```


&nbsp;


We go step-wise through the python code. First, a parameter-server for the simulation has to be defined. If we do not pass a parameter-file, default arguments are being used.
```python
param_server = ParameterServer()
```

Next, the world object using the parameter-server is created.
```python
world = World(param_server)
```

In order to generate a behavior, validate it and for it to be dynamically feasible we define the models as [described](agent_components.md).
```python
behavior_model = BehaviorConstantVelocity(param_server)
execution_model = ExecutionModelInterpolate(param_server)
dynamic_model = SingleTrackModel()
```

The map is specified using:
```python
xodr_parser = XodrParser("modules/runtime/tests/data/Crossing8Course.xodr")
```

Furthermore, the `XodrParser` generates the `OpenDriveMap` and `Roadgraph`. These generated entities are then passed to the `MapInterface`.
```python
map_interface = MapInterface()
map_interface.SetOpenDriveMap(xodr_parser.map)
map_interface.SetRoadgraph(xodr_parser.roadgraph)
world.SetMap(map_interface)
```

Next, our environment needs agents! Therefore, we initiate our agent with a vehicle-shape, a new parameter and an initial-state `x = [t, x, y, theta, v]`.
Furthermore, we set the previously initiated models (behavior, execution and dynamic) to the agent.

```python
agent_2d_shape = CarLimousine()
init_state = np.array([0, -11, -8, 3.14*3.0/4.0, 50/3.6]) 
agent_params = param_server.addChild("agent1")
agent = Agent(init_state, behavior_model, dynamic_model, execution_model, agent_2d_shape, agent_params, 2, map_interface)
world.AddAgent(agent)
```

In order to have insights on how our agent act we can specify a viewer. Here we choose a simple, real-time 2D-viewer (PygameViewer).
If required, the MatplotlibViewer can be used in order to generate publication-ready figures.

```python
viewer = PygameViewer(params=param_server, x_range=[-20, 20], y_range=[-200, 20], follow_agent_id=agent.id)
```

Finally, to run the simulation use the following lines:
```python
sim_step_time = param_server["simulation"]["step_time", "Gives the amount of time in which one behavior planning call has to produce the result", 1]
sim_real_time_factor = param_server["simulation"]["real_time_factor", "How much faster than real-time, simulation shall be played", 1]

for _ in range(0, 30):
    world.Step(sim_step_time)
    viewer.drawWorld(world)
    viewer.show(block=False)
    time.sleep(sim_step_time/sim_real_time_factor)
```

In order to make the experiment reproducible, use:
```python
param_server.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "params", "od8_const_vel_one_agent_written.json"))
```

