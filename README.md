<p align="center">
<img src="https://github.com/bark-simulator/bark/raw/master/docs/source/bark_logo.jpg" alt="BARK" />
</p>

![Ubtuntu-CI Build](https://github.com/bark-simulator/bark/workflows/CI/badge.svg)
![macOS-CI Build](https://github.com/bark-simulator/bark/workflows/macOS-CI/badge.svg)
![NIGHTLY Build](https://github.com/bark-simulator/bark/workflows/NIGHTLY/badge.svg)
![NIGHTLY LTL Build](https://github.com/bark-simulator/bark/workflows/NIGHTLY_LTL/badge.svg)

# BARK - a tool for **B**ehavior benchm**ARK**ing

BARK is a semantic simulation framework for autonomous agents with a special focus on autonomous driving.
Its behavior model-centric design allows for the rapid development, training and benchmarking of various decision-making algorithms.
Due to its fast, semantic runtime, it is especially suited for computationally expensive tasks, such as reinforcement learning.

## BARK Ecosystem

The BARK ecosystem is composed of multiple components that all share the common goal to develop and benchmark behavior models:

* [BARK-ML](https://github.com/bark-simulator/bark-ml/): Machine learning library for decision-making in autonomous driving.
* [BARK-MCTS](https://github.com/bark-simulator/planner-mcts): Integrates a template-based C++ Monte Carlo Tree Search Library into BARK to support development of both single- and multi-agent search methods.

* [BARK-DB](https://github.com/bark-simulator/bark-databasse/): Provides a framework to integrate multiple BARK scenario sets into a database. The database module supports binary seriliazation of randomly generated scenarios to ensure exact  reproducibility of behavior benchmarks accross systems. 
* [CARLA-Interface](https://github.com/bark-simulator/carla-interface): A two-way interface between [CARLA ](https://github.com/carla-simulator/carla) and BARK. BARK behavior models can control CARLA vehicles. CARLA controlled vehicles are mirrored to BARK.

## Quick Start

Use `git clone https://github.com/bark-simulator/bark.git` or download the repository from this page.
Then follow the instructions at [How to Install BARK](https://github.com/bark-simulator/bark/blob/master/docs/source/installation.md).
Once you activated the virtual environment (`source dev_into.sh`), you can explore some examples of BARK.

Highway: `bazel run //examples:highway`:
<p align="center">
<img src="https://github.com/bark-simulator/bark/raw/master/docs/source/gifs/bark_highway.gif" alt="BARK" />
</p>

Merging: `bazel run //examples:merging`:
<p align="center">
<img src="https://github.com/bark-simulator/bark/raw/master/docs/source/gifs/bark_merging.gif" alt="BARK" />
</p>

Intersection: `bazel run //examples:intersection`:
<p align="center">
<img src="https://github.com/bark-simulator/bark/raw/master/docs/source/gifs/bark_intersection.gif" alt="BARK" />
</p>

To get step-by-step instructions on how to use BARK, you can run our IPython Notebook tutorials using `bazel run //docs/tutorials:run`.
For a more detailed understanding of how BARK works, its concept and use cases have a look at our [documentation](https://bark-simulator.readthedocs.io/en/latest/about.html).

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

## License

BARK specific code is distributed under MIT License.
