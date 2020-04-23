# Tutorials

## Beginner
- Bark Concept: shortly explain bark's concept and link to the bark paper, how to construct an agent (explain model types, shape, state), add several agents to the world, how to setup the highway map interface and do a small simulation run with multiple steps (example opendrive8 but with highway)
- Write a python behavior model: python system test behavior model
- Benchmarking Toolchain: Explain BenchmarkDatabase and runner, run benchmark, small evaluation, use the example python agent of previous tutorial to benchmark and compare it against idm model (python system test behavior model + benchmark runner test)
- ConfigurableScenarioGeneration: parameterize different scenarios, highway, lon lat distances
- InteractionDataSetScenarioGeneration: show how to load interaction dataset and replace agents

## Advanced
- Bark-ML: Show how to configure observers and evalutors, show how to train and evaluate different RL algorithms 
- MCTS: Understand current mcts implementation and parameters, do small benchmark for different numbers of iterations, varying prediction models