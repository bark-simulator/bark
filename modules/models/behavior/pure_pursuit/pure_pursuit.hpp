#ifndef MODULES_MODELS_BEHAVIOR_PURE_PURSUIT_PURE_PURSUIT_HPP_
#define MODULES_MODELS_BEHAVIOR_PURE_PURSUIT_PURE_PURSUIT_HPP_

#include "modules/geometry/line.hpp"
#include "modules/models/behavior/behavior_model.hpp"
#include "modules/models/dynamic/single_track.hpp"


namespace modules {
namespace models {
namespace behavior {

using world::ObservedWorld;

// From technical report "Implementation of the Pure Pursuit Path Tracking Algorithm"
class BehaviorPurePursuit : public BehaviorModel {
  public:
    explicit BehaviorPurePursuit(commons::Params *params) :
      BehaviorModel(params) {}
    
    virtual ~BehaviorPurePursuit() {}

    Trajectory Plan(float delta_time,
                    const ObservedWorld& observed_world);

    void set_followed_line(const geometry::Line followed_line) { followed_line_ = followed_line; }

    virtual BehaviorModel *Clone() const;
  
  private:
    float FindSteeringAngle(const dynamic::State &agent_state, const geometry::Line &line) const;

    dynamic::SingleTrackModel single_track_model_;
    geometry::Line followed_line_;
};

inline BehaviorModel *BehaviorPurePursuit::Clone() const {
  return new BehaviorPurePursuit(*this);
}

} // namespace behavior
} // namespace models
} // namespace modules

#endif // MODULES_MODELS_BEHAVIOR_PURE_PURSUIT_PURE_PURSUIT_HPP_