<p align="center">
<img src="docs/source/bark_logo.jpg" alt="BARK" />
</p>

![CI Build](https://github.com/bark-simulator/bark/workflows/CI/badge.svg)
![NIGHTLY Build](https://github.com/bark-simulator/bark/workflows/NIGHTLY/badge.svg)

# BARK - a tool for **B**ehavior benchm**ARK**ing
BARK is a semantic simulation framework for autonomous agents with a special focus on autonomous driving.
Its behavior model-centric design allows for the rapid development, training and benchmarking of various decision-making algorithms.
Due to its fast, semantic runtime, it is especially suited for computationally expensive tasks, such as reinforcement learning.


## BARK Ecosystem
The BARK ecosystem is composed of multiple components that all share the common goal to develop and benchmark behavior models.

The currently available modules are:
* [BARK-ML](https://github.com/bark-simulator/bark-ml/): Machine learning library for BARK
* [BARK-DB](https://github.com/bark-simulator/bark-databasse/): Database with serialized scenarios for benchmarking of behavior models
* [CARLA-Interface](https://github.com/bark-simulator/carla-interface): Interface with Carla enabling BARK models


## Paper
If you use BARK, please cite us using the following paper:

```
@inproceedings{BARK,
  title = {BARK: Open Behavior Benchmarking in Multi-Agent Environments},
  author = {Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler},
}
```


## Quick Start
Use `git clone https://github.com/bark-simulator/bark.git` or download the repository from this page.
Then follow the instructions at [How to Install BARK](https://github.com/bark-simulator/bark/blob/master/docs/source/installation.md).

After the installation, you can explore the examples by e.g. running `source dev_into.sh && bazel run //examples:od8_const_vel_two_agent`.


For a more detailed understanding of how BARK works, its concept and use cases have a look at our [documentation](https://bark-simulator.readthedocs.io/en/latest/about.html).


## License
BARK specific code is distributed under MIT License.