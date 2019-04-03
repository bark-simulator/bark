// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_LANE_HPP_
#define MODULES_WORLD_OPENDRIVE_LANE_HPP_

#include <string>
#include <vector>
#include <map>
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/world/opendrive/commons.hpp"

namespace modules {
namespace world {
namespace opendrive {


class Lane {
 public:
  Lane() : lane_id_(++lane_count), lane_position_(0),
           link_(), line_(), road_mark_(), speed_() {}
  explicit Lane(const LanePosition& lane_position) : lane_id_(++lane_count),
   lane_position_(lane_position), link_(), line_(), road_mark_(), speed_() {}

  ~Lane() {}

  //! setter functions
  void set_id(const LaneId lane_id) { lane_id_ = lane_id; }
  void set_line(const geometry::Line line) { line_ = line; }
  void set_link(const Link link) { link_ = link; }
  void set_speed(float speed) { speed_ = speed; }
  void set_road_mark(const RoadMark rm) { road_mark_ = rm; }
  void set_lane_position(const LanePosition& lane_position)
                             { lane_position_ = lane_position; }

  //! getter functions
  geometry::Line get_line() { return line_; }

  Link get_link() const { return link_; }
  RoadMark get_road_mark() const { return road_mark_; }
  float get_speed() const { return speed_; }
  LaneId get_id() const { return lane_id_; }
  LanePosition get_lane_position() const { return lane_position_;}

  float curvature_at(const float s, const float s_delta = 0.01) const;
  float curvature_dot_at(const float s) const;
  float lane_width_at(const float s) const;
  float s_from_point(const geometry::Point2d &point) const;


 private:
  LaneId lane_id_;
  LanePosition lane_position_;
  Link link_;
  geometry::Line line_;

  RoadMark road_mark_;
  float speed_;

  static LaneId lane_count;
};

inline std::string print(const Lane &l) {
  std::stringstream ss;
  ss << "id: " << l.get_id();
  ss << print(l.get_link());
  ss << print(l.get_road_mark());
  ss << "speed: " << l.get_speed() << std::endl;
  return ss.str();
}

using LanePtr = std::shared_ptr<Lane>;
using LaneSequence = std::vector<LaneId>;
using LaneSequences = std::vector<LaneSequence>;
using Lanes = std::map<LaneId, LanePtr>;


}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_LANE_HPP_
