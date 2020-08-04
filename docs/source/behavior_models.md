Behavior Models
================================
Overview of all behavior models available in BARK.


## Constant Velocity Model

The most basic model available in BARK, is the constant velocity behavior model.
The `BehaviorConstantAcceleration` class interpolates an agent along a set line with a constant velocity.
There are no collision checks and, thus, vehicles with different speeds can collide with each other.


## Intelligent Driver Model

A more advanced behavior model in BARK is the intelligent driver model (IDM).
The `BaseIDM` class is the base class of all IDM models.
It provides all functions required for the calculation of the acceleration.

### Classic

Similar to the constant velocity model, the `BehaviorIDMClassic` interpolates itself along a line.
Additionally, it changes the acceleration based on the free-road- and interaction-term to model realistic lane-following behavior.
Thus, if the initial conditions are feasible, the IDM normally does not cause any collisions.


### Lane Tracking

The `BehaviorIDMLaneTracking` is similar to the classic IDM model but uses a dynamic model and steering function to follow a line.


## Mobil Model

The Mobil model extends the IDM model even further.
It additionally checks, whether a lane-change would be beneficial for the ego vehicle as well as for the surrounding vehicles.
If the politeness parameter of the Mobil model is set to zero, it only checks how to proceed the fastest on a road (mult. lanes).


## Rule-based Models

The rule-based models in BARK have been developed to allow more sophisticated behaviors, such as braking, changing lanes, and handling intersections.
One of the core concepts of these models is a filter function that allows formulating lambda-functions to sort out lane-corridors.
This filtering is versatile and e.g. can use the free-space on the other lane or more sophisticated methods.


### Lane Change Model

There are two rule-based lane-changing models currently implemented in BARK: `BehaviorLaneChangeRuleBased` and `BehaviorMobilRuleBased`.
The `BehaviorLaneChangeRuleBased` class checks the free-space on all available lanes and changes to the one having the most free space.
The `BehaviorMobilRuleBased` class acts as the normal Mobil model, but only can change to filtered lanes (e.g. that have sufficient free-space).


### Intersection Model

The intersection model can handle intersections of arbitrary shape.
It is prediction- and rule-based.
If an agent intersects the ego agent's `LaneCorridor` first, the ego agent has to brake.
However, to avoid deadlocks (if both agents intersect at the same time), there is an additional right before left rule.
This model is not based on any literature but has empirically shown to work well.


## Behavior Dynamic Model

This model is an externally controlled model, e.g. by a neural-network and requires the action to be set.
Once the action is set, the `Step` function of the BARK world can be called and the `BehaviorDynamicModel` will produce a trajectory using the set action.
This model is e.g. used in [BARK-ML](https://github.com/bark-simulator/bark-ml).


## Behavior Motion Primitives

In BARK there are macro and continuous motion primitives.
The macro motion primitives have actions, such as follow the lane or change the lane to the left.
The continous motion primitives use continuous action inputs and push these to a vector.