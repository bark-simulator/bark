Behavior Models
================================
A short overview over the behavior models available in BARK.

## BehaviorConstantVelocity

The constant-velocity model interpolates an agent along a set route with constant velocity.

```cpp
class BehaviorConstantVelocity : public BehaviorModel {
 public:
  explicit BehaviorConstantVelocity(const commons::ParamsPtr& params) :
    BehaviorModel(params) {}

  virtual ~BehaviorConstantVelocity() {}

  Trajectory Plan(AgentId agent_id,
                 float delta_time,
                 const ObservedWorld& observed_world);

  virtual BehaviorModel *Clone() const;
};
```

## IntelligentDriverModel

## Mobil

## Rule-based
### Intersection
### Intersection