<p align="center">
<img src="https://github.com/bark-simulator/bark/raw/master/docs/source/bark_logo.jpg" alt="BARK" />
</p>

![Ubtuntu-CI Build](https://github.com/bark-simulator/bark/workflows/CI/badge.svg)
![Ubtuntu-ManyLinux Build](https://github.com/bark-simulator/bark/workflows/ManyLinux/badge.svg)
![NIGHTLY LTL Build](https://github.com/bark-simulator/bark/workflows/NIGHTLY_LTL/badge.svg)
![CI RSS Build](https://github.com/bark-simulator/bark/workflows/CI_RSS/badge.svg)
![NIGHTLY Rules MCTS Build](https://github.com/bark-simulator/bark/workflows/NIGHTLY_RULES_MCTS/badge.svg)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/b9f484c42194487e9b9b33742381e992)](https://www.codacy.com/gh/bark-simulator/bark/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=bark-simulator/bark&amp;utm_campaign=Badge_Grade)
# BARK - A Tool for **B**ehavior benchm**ARK**ing

BARK is a semantic simulation framework for autonomous driving. Its behavior model-centric design allows for the rapid development, training, and benchmarking of various decision-making algorithms. It is especially suited for computationally expensive tasks, such as reinforcement learning. A a good starting point, have a look at the content of our [BARK-Tutorial on IROS 2020](https://bark-simulator.github.io/tutorials/).


## Usage

### (A) Pip Package

*For whom it is: Python evangelists implementing python behavior models or ML scientists using BARK-ML for learning behaviors.*

Bark is available as  [PIP-Package](https://pypi.org/project/bark-simulator/) for Ubuntu and MacOS for Python>=3.7. You can install the latest version with 
`pip install bark-simulator`. The Pip package supports full benchmarking functionality of existing behavior models and development of your models within python.

After installing the package, you can have a look at the [examples](https://github.com/bark-simulator/bark/tree/master/bark/examples) to check how to use BARK. 

| Highway Example | Merging Example | Intersection Example |
| --- | --- | --- |
| ![Intersection](https://github.com/bark-simulator/bark/raw/master/docs/source/gifs/bark_highway.gif) | ![Intersection](https://github.com/bark-simulator/bark/raw/master/docs/source/gifs/bark_merging.gif) | ![Intersection](https://github.com/bark-simulator/bark/raw/master/docs/source/gifs/bark_intersection.gif) |
| `import bark.examples.highway` | `import bark.examples.merging` | `import bark.examples.intersection` |


### (B) Build it from Source

*For whom it is: C++ developers creating C++ behavior models, researchers performing benchmarks, or contributors to BARK.*

Use `git clone https://github.com/bark-simulator/bark.git` or download the repository from this page.
Then follow the instructions at [How to Install BARK](https://github.com/bark-simulator/bark/blob/master/docs/source/installation.md).

To get step-by-step instructions on how to use BARK, you can run our [IPython Notebook tutorials](https://github.com/bark-simulator/bark/tree/master/docs/tutorials) using `bazel run //docs/tutorials:run`.
For a more detailed understanding of how BARK works, its concept and use cases have a look at our [documentation](https://bark-simulator.readthedocs.io/en/latest/about.html).

[Example Benchmark](https://github.com/bark-simulator/example_benchmark) is a running example of how to use BARK for benchmarking for scientific purposes.


## Scientific Publications using BARK

*   [BARK: Open Behavior Benchmarking in Multi-Agent Environments](https://arxiv.org/abs/2003.02604) (IROS 2020)
*   [Graph Neural Networks and Reinforcement Learning for Behavior Generation in Semantic Environments](https://arxiv.org/abs/2006.12576) (IV 2020)
*   [Counterfactual Policy Evaluation for Decision-Making in Autonomous Driving](https://arxiv.org/abs/2003.11919) (IROS 2020,  PLC Workshop)
*   [Modeling and Testing Multi-Agent Traffic Rules within Interactive Behavior Planning](https://arxiv.org/abs/2009.14186) (IROS 2020,  PLC Workshop)
*   [Formalizing Traffic Rules for Machine Interpretability](https://arxiv.org/abs/2007.00330) (CAVS 2020)
*   [Robust Stochastic Bayesian Games for Behavior Space Coverage](https://arxiv.org/abs/2003.11281) (RSS 2020, Workshop on Interaction and Decision-Making in Autonomous-Driving)
*   [Risk-Constrained Interactive Safety under Behavior Uncertainty for Autonomous Driving](https://arxiv.org/abs/2102.03053) (IV 2021)


## BARK Ecosystem

The BARK ecosystem is composed of multiple components that all share the common goal to develop and benchmark behavior models:

* [BARK-ML](https://github.com/bark-simulator/bark-ml/): Machine learning library for decision-making in autonomous driving.
* [BARK-MCTS](https://github.com/bark-simulator/planner-mcts): Integrates a template-based C++ Monte Carlo Tree Search Library into BARK to support development of both single- and multi-agent search methods.
* [BARK-Rules-MCTS](https://github.com/bark-simulator/planner-rules-mcts): Integrates traffic rules within Monte Carlo Tree Search with lexicographic ordering.
* [BARK-DB](https://github.com/bark-simulator/bark-databasse/): Provides a framework to integrate multiple BARK scenario sets into a database. The database module supports binary serialization of randomly generated scenarios to ensure exact reproducibility of behavior benchmarks across systems. 
* [BARK-Rule-Monitoring](https://github.com/bark-simulator/rule-monitoring): Provides runtime verification of Rules in Linear Temporal Logic (LTL) on simulated BARK traces.
* [CARLA-Interface](https://github.com/bark-simulator/carla-interface): A two-way interface between [CARLA ](https://github.com/carla-simulator/carla) and BARK. BARK behavior models can control CARLA vehicles. CARLA controlled vehicles are mirrored to BARK.


## Paper

If you use BARK, please cite us using the following [paper](https://arxiv.org/abs/2003.02604):

```
@inproceedings{Bernhard2020,
    title = {BARK: Open Behavior Benchmarking in Multi-Agent Environments},
    author = {Bernhard, Julian and Esterle, Klemens and Hart, Patrick and Kessler, Tobias},
    booktitle = {2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
    url = {https://arxiv.org/pdf/2003.02604.pdf},
    year = {2020}
}
```


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.


## License

BARK specific code is distributed under [MIT](https://choosealicense.com/licenses/mit/) License.
