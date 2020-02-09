Runtime
==================================

The runtime module implements the actual simulation in Python. Currently, it uses a simple step-based loop:

```python
class Runtime:
    def __init__(self, world, step_time, viewer):
        self.world = world
        self.Step_time = step_time
	self.viewer = viewer

    def run(self, steps):
        for step_count in trange(steps, leave=True):
            self.world.Step(self.Step_time)
	    self.viewer.drawWorld(self.world)
	    sleep(...)
```

In the future, a fully featured simulation with a benchmarking suite is planned.


## Viewer

A common viewer interface allows easy extension of visualization capabilities of Bark. Currently, we provide a real-time capable, 2D visualization based on PyGame (PygameViewer) and for single-scene shots one based on Matplotlib (MatplotlibViewer).



