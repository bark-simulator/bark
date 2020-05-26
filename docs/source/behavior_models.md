Behavior Models
================================
Short overview of the behavior models available in BARK.

## Constant Velocity Model

The `BehaviorConstantVelocity` class interpolates an agent along a set route with constant velocity.

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

## Intelligent Driver Model


## Mobil Model


## Rule-based Models

### Lane Change Model

### Intersection Model

