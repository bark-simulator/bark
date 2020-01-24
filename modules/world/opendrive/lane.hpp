// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_OPENDRIVE_LANE_HPP_
#define MODULES_WORLD_OPENDRIVE_LANE_HPP_

#include <map>
#include <string>
#include <vector>
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/world/opendrive/commons.hpp"

namespace modules {
namespace world {
namespace opendrive {

class XodrLane {
 public:
  XodrLane()
      : lane_id_(++lane_count),
        lane_position_(0),
        link_(),
        line_(),
        lane_type_(XodrLaneType::NONE),
        driving_direction_(XodrDrivingDirection::FORWARD),
        road_mark_(),
        speed_() {}
  explicit XodrLane(const XodrLanePosition& lane_position)
      : lane_id_(++lane_count),
        lane_position_(lane_position),
        link_(),
        line_(),
        lane_type_(XodrLaneType::NONE),
        driving_direction_(XodrDrivingDirection::FORWARD),
        road_mark_(),
        speed_() {}

  explicit XodrLane(const std::shared_ptr<XodrLane>& lane) :
    lane_id_(lane->lane_id_),
    lane_position_(lane->lane_position_),
    link_(lane->link_),
    line_(lane->line_),
    lane_type_(lane->lane_type_),
    driving_direction_(lane->driving_direction_),
    road_mark_(lane->road_mark_),
    speed_(lane->speed_) {}

  ~XodrLane() {}

  //! setter functions
  void SetId(const XodrLaneId lane_id) { lane_id_ = lane_id; }
  void set_line(const geometry::Line line) { line_ = line; }
  void SetLink(const XodrLaneLink link) { link_ = link; }
  void set_speed(float speed) { speed_ = speed; }
  void set_lane_type(const XodrLaneType lt) { lane_type_ = lt; }
  void set_driving_direction(const XodrDrivingDirection& d) {
    driving_direction_ = d;}
  void set_road_mark(const XodrRoadMark rm) { road_mark_ = rm; }
  void set_lane_position(const XodrLanePosition& lane_position) {
    lane_position_ = lane_position;
  }

  bool append(geometry::Line previous_line, XodrLaneWidth lane_width_current,
              float s_inc);

  //! getter functions
  geometry::Line get_line() const { return line_; }

  XodrLaneLink GetLink() const { return link_; }
  XodrRoadMark get_road_mark() const { return road_mark_; }
  float get_speed() const { return speed_; }
  XodrLaneType GetLane_type() const { return lane_type_; }
  XodrDrivingDirection get_driving_direction() const {
    return driving_direction_; }
  XodrLaneId GetId() const { return lane_id_; }
  XodrLanePosition GetLane_position() const { return lane_position_; }

  float curvature_at(const float s, const float s_delta = 0.01) const;
  float curvature_dot_at(const float s) const;
  float lane_width_at(const float s) const;
  float s_from_point(const geometry::Point2d& point) const;

 private:
  XodrLaneId lane_id_;
  XodrLanePosition lane_position_;
  XodrLaneLink link_;
  geometry::Line line_;

  XodrLaneType lane_type_;
  XodrDrivingDirection driving_direction_;
  XodrRoadMark road_mark_;
  float speed_;

  static XodrLaneId lane_count;
};

inline std::string print(const XodrLane& l) {
  std::stringstream ss;
  ss << "id: " << l.GetId() << ", ";
  ss << "position " << l.GetLane_position() << ", ";
  ss << print(l.GetLink());
  ss << print(l.get_road_mark());
  ss << "speed: " << l.get_speed() << std::endl;
  return ss.str();
}

using XodrLanePtr = std::shared_ptr<XodrLane>;
using XodrLaneSequence = std::vector<XodrLaneId>;
using XodrLaneSequences = std::vector<XodrLaneSequence>;
using XodrLanes = std::map<XodrLaneId, XodrLanePtr>;

inline XodrLanePtr create_lane_from_lane_width(XodrLanePosition lane_position,
                                           geometry::Line previous_line,
                                           XodrLaneWidth lane_width_current,
                                           float s_inc = 0.5f) {
  std::shared_ptr<XodrLane> ret_lane(new XodrLane(lane_position));

  bool succ = ret_lane->append(previous_line, lane_width_current, s_inc);

  return ret_lane;
}

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_LANE_HPP_
