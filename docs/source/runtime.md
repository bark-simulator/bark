Runtime
==================================

The runtime module implements the actual simulation in Python.
It provides a similar interface as the [OpenAI Gym](https://gym.openai.com/) environments.

```python
class Runtime(PyRuntime):
  def __init__(self,
               step_time,
               viewer,
               scenario_generator=None,
               render=False):
    self._step_time = step_time
    self._viewer = viewer
    self._scenario_generator = scenario_generator
    self._scenario_idx = None
    self._scenario = None
    self._render = render
    ...

  def reset(self, scenario=None):
    ...

  def step(self):
    ...

  def render(self):
    ...
```

The runtime has a scenario generator that fills in the `self._scenario` and ID of the current scenario `self._scenario_idx`.


## Scenario

The scenario fully defines the initial state of the simulation, such as the agent's positions and models.

The outline of the `Scenario` class is given by:

```python
class Scenario:
  def __init__(self,
               agent_list=None,
               eval_agent_ids=None,
               map_file_name=None,
               json_params=None,
               map_interface=None):
    self._agent_list = agent_list or []
    self._eval_agent_ids = eval_agent_ids or []
    self._map_file_name = map_file_name
    self._json_params = json_params
    self._map_interface = map_interface
  ...
```

It also specifies which agents should be evaluated using `self._eval_agent_ids`.


## Scenario Generation

A scenario generation in BARK returns a list of scenarios of the type  `Scenario`.
These can be run by the BARK runtime or by the `BenchmarkRunner`.

Currently available scenario generators:

* `ConfigurableScenarioGeneration`: Sophisticated scenario generation providing conflict resolution.
* `UniformVehicleDistribution`: Samples the agents uniformly and their parameters.
* `ConfigWithEase`: Configure any scenario fast and with ease.
* `DeterministicScenarioGeneration`: Deterministic, reproducible scenario generation.


## Benchmarking

BARK provides a `BenchmarkRunner` and `BenchmarkAnalyzer` to automatically run and verify the performance of novel behavior models.


## Viewer

A common viewer interface allows easy extension of visualization capabilities of Bark.

Several viewer modules are currently available:

* `MPViewer`: Matplotlib viewer for scientific documentation.
* `Panda3dViewer`: 3D-Visualization.
* `PygameViewer`: Gym-like visualization of the BARK environment.
