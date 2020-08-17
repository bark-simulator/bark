Examples
=========================
To get started with BARK, we provide several examples that show the basic functionality.
All examples are found in the `/examples`-directory of BARK.


## Merging Example

In this example, we show the basic functionality of BARK using a merging scenario.
It can be ran using: `bazel run //examples:merging`.

```eval_rst
.. image:: gifs/bark_merging.gif
   :scale: 100 %
   :align: left
```

BARK uses a `ParameterServer()` that stores all parameters of the simulation.
We can set parameters globally:

```python
param_server["BehaviorIDMClassic"]["DesiredVelocity"] = 10.
```

We define the scenario using a scenario generation module of BARK.
In the following example, both lanes of the merging scenario are defined with a controlled agent on the right lane.

```python
# configure both lanes
left_lane = CustomLaneCorridorConfig(params=param_server,
                                     lane_corridor_id=0,
                                     road_ids=[0, 1],
                                     behavior_model=BehaviorMobilRuleBased(param_server),
                                     s_min=0.,
                                     s_max=50.)
right_lane = CustomLaneCorridorConfig(params=param_server,
                                      lane_corridor_id=1,
                                      road_ids=[0, 1],
                                      controlled_ids=True,
                                      behavior_model=BehaviorMobilRuleBased(param_server),
                                      s_min=0.,
                                      s_max=20.)

scenarios = \
  ConfigWithEase(num_scenarios=3,
                 map_file_name="bark/runtime/tests/data/DR_DEU_Merging_MT_v01_shifted.xodr",
                 random_seed=0,
                 params=param_server,
                 lane_corridor_configs=[left_lane, right_lane])
```

We then define the viewer and runtime in order to run and visualize the scenarios:

```python
viewer = MPViewer(params=param_server,
                  x_range=[-35, 35],
                  y_range=[-35, 35],
                  follow_agent_id=True)
env = Runtime(step_time=0.2,
              viewer=viewer,
              scenario_generator=scenarios,
              render=True)
```

Running scenarios can now be easily done as follows:

```python
# run 3 scenarios
for _ in range(0, 3):
  env.reset()
  for step in range(0, 90):
    env.step()
    time.sleep(sim_step_time/sim_real_time_factor)
```

However, BARK also provides a `BenchmarkRunner` that runs scenarios automatically and benchmarks the performance of behavior models.



## Other Examples
The other examples can be run in a similar fashion using:

* `bazel run //examples:highway`: Two-lane highway example.
* `bazel run //examples:intersection`: Three way intersection.
* `bazel run //examples:interaction_dataset`: Dataset replay.
* `bazel run //examples:benchmark_database`: Benchmarks behaviors using a scenario database.
