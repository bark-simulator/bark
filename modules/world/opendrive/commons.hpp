// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#ifndef MODULES_WORLD_OPENDRIVE_COMMONS_HPP_
#define MODULES_WORLD_OPENDRIVE_COMMONS_HPP_

#include <inttypes.h>
#include <string>
#include <vector>


namespace modules {
namespace world {
namespace opendrive {


using LaneId = uint32_t;
using LanePosition = int32_t;
using RoadId = uint32_t;

struct LinkInfo {
  LinkInfo() : position_(1000000), type_("") {}
  LinkInfo(const LanePosition& pos, const std::string& type) : position_(pos), type_(type) {}
  LanePosition position_;
  std::string type_;
};

struct Link {
  Link() : predecessor_(), successor_() {}
  Link(const LinkInfo& predecessor, const LinkInfo& successor) : predecessor_(predecessor), successor_(successor) {}
  LinkInfo predecessor_;
  LinkInfo successor_;
  //! getter
  LinkInfo get_predecessor() const { return predecessor_; }
  LinkInfo get_successor() const { return successor_; }
  void set_predecessor(const LinkInfo& info) { predecessor_ = info; }
  void set_successor(const LinkInfo& info) { successor_ = info; }
};

inline std::string print(const Link &l) {
  std::stringstream ss;
  ss << "Link.predecessor: " << l.predecessor_.position_ << "of type" << l.predecessor_.type_ << "; ";
  ss << "Link.successor: " << l.successor_.position_ << "of type" << l.successor_.type_ << std::endl;
  return ss.str();
}

struct LaneOffset {
  float a, b, c, d;
};

inline float polynom(float x, float a, float b, float c, float d) {
  return a + b * x + c * x * x + d * x * x * x;
}

struct LaneLink {
  int from_id;
  int to_id;
};

using LaneLinks = std::vector<LaneLink>;

struct Connection {
  void add_lane_link(LaneLink link) { lane_links_.push_back(link); }
  LaneLinks get_lane_links() const { return lane_links_; }
  uint32_t id_;
  uint32_t incoming_road_;
  uint32_t connecting_road_;
  LaneLinks lane_links_;
};

struct RoadMark {
  std::string type_;
  std::string color_;
  float width_;
};

inline std::string print(const RoadMark &r) {
  std::stringstream ss;
  ss << "RoadMark: type: " << r.type_ << ", id: " << r.color_ << ", width: " << r.width_ << std::endl;
  return ss.str();
}

struct LaneWidth {
  float s_start;
  float s_end;
  LaneOffset off;
};

using LaneWidths = std::vector<LaneWidth>;

}  // namespace opendrive
}  // namespace world
}  // namespace modules

#endif  // MODULES_WORLD_OPENDRIVE_COMMONS_HPP_
