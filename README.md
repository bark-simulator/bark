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
The BARK ecosystem is composed of multiple components that all share the common goal to develop and benchmark behavior models:

* [BARK-ML](https://github.com/bark-simulator/bark-ml/): Develop behavior models based on machine learning library.
* [BARK-MCTS](https://github.com/bark-simulator/planner-mcts): Integrates a template-based C++ Monte Carlo Tree Search Library into BARK to support development of both single- and multi-agent search methods.

* [BARK-DB](https://github.com/bark-simulator/bark-databasse/): Provides a framework to integrate multiple BARK scenario sets into a database. The database module supports binary seriliazation of randomly generated scenarios to ensure exact  reproducibility of behavior benchmarks accross systems. 
* [CARLA-Interface](https://github.com/bark-simulator/carla-interface): A two-way interface between [CARLA ](https://github.com/carla-simulator/carla) and BARK. BARK behavior models can control CARLA vehicles. CARLA controlled vehicles are mirrored to BARK.


## Paper
If you use BARK, please cite us using the following paper:

```
@misc{bernhard2020bark,
    title={BARK: Open Behavior Benchmarking in Multi-Agent Environments},
    author={Julian Bernhard and Klemens Esterle and Patrick Hart and Tobias Kessler},
    year={2020},
    eprint={2003.02604},
    archivePrefix={arXiv},
    primaryClass={cs.MA}
}
```


## Quick Start
Use `git clone https://github.com/bark-simulator/bark.git` or download the repository from this page.
Then follow the instructions at [How to Install BARK](https://github.com/bark-simulator/bark/blob/master/docs/source/installation.md).

After the installation, you can explore the examples by e.g. running `source dev_into.sh && bazel run //examples:od8_const_vel_two_agent`.

<p align="center">
<img src="docs/source/example_map.gif" alt="BARK" />
</p>

To get step-by-step instructions on how to use BARK, you can run our IPython Notebook tutorials using `bazel run //docs/tutorials:run`.
For a more detailed understanding of how BARK works, its concept and use cases have a look at our [documentation](https://bark-simulator.readthedocs.io/en/latest/about.html).


## License
BARK specific code is distributed under MIT License.
