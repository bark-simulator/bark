// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_WORLD_OPENDRIVE_COMMONS_HPP_
#define BARK_WORLD_OPENDRIVE_COMMONS_HPP_

#include <inttypes.h>
#include <string>
#include <vector>

#include <boost/geometry.hpp>
#include "bark/geometry/line.hpp"

namespace bark {
namespace world {
namespace opendrive {

using XodrLaneId = uint32_t;
using XodrLanePosition = int32_t;
using XodrRoadId = uint32_t;

struct XodrRoadLinkInfo {
  XodrRoadLinkInfo() : id_(1000000), type_("") {}
  XodrRoadLinkInfo(const XodrRoadId& id, const std::string& type)
      : id_(id), type_(type) {}
  XodrRoadId id_;
  std::string type_;
};

struct XodrRoadLink {
  XodrRoadLink() : predecessor_(), successor_() {}
  XodrRoadLink(const XodrRoadLinkInfo& predecessor,
               const XodrRoadLinkInfo& successor)
      : predecessor_(predecessor), successor_(successor) {}
  XodrRoadLinkInfo predecessor_;
  XodrRoadLinkInfo successor_;
  //! getter
  XodrRoadLinkInfo GetPredecessor() const { return predecessor_; }
  XodrRoadLinkInfo GetSuccessor() const { return successor_; }
  void SetPredecessor(const XodrRoadLinkInfo& info) { predecessor_ = info; }
  void SetSuccessor(const XodrRoadLinkInfo& info) { successor_ = info; }
};

inline std::string print(const XodrRoadLink& l) {
  std::stringstream ss;
  ss << "XodrRoadLink.predecessor: " << l.predecessor_.id_ << "of type"
     << l.predecessor_.type_ << "; ";
  ss << "XodrRoadLink.successor: " << l.successor_.id_ << "of type"
     << l.successor_.type_ << std::endl;
  return ss.str();
}

struct XodrLaneOffset {
  float a, b, c, d;
};

inline float Polynom(float x, float a, float b, float c, float d) {
  return a + b * x + c * x * x + d * x * x * x;
}

// TODO(@all): use type XodrLaneId here
struct XodrLaneLink {
  XodrLanePosition from_position;
  XodrLanePosition to_position;
};

inline std::string print(const XodrLaneLink& l) {
  std::stringstream ss;
  ss << "XodrLaneLink.from_position: " << l.from_position << "; ";
  ss << "XodrLaneLink.to_position: " << l.to_position << std::endl;
  return ss.str();
}

using XodrLaneLinks = std::vector<XodrLaneLink>;

struct Connection {
  void AddLaneLink(XodrLaneLink link) { lane_links_.push_back(link); }
  XodrLaneLinks GetLaneLinks() const { return lane_links_; }
  uint32_t id_;
  uint32_t incoming_road_;  // TODO(@all): use type XodrRoadId here
  uint32_t connecting_road_;
  XodrLaneLinks lane_links_;
};

enum XodrDrivingDirection { FORWARD = 0, BACKWARD = 1, BOTH = 2 };

enum XodrLaneType {
  NONE = 0,
  DRIVING = 1,
  // STOP = 2,
  // SHOULDER = 3,
  BIKING = 4,
  SIDEWALK = 5,
  BORDER = 6,
  /*RESTRICTED = 7,
  PARKING = 8,
  BIDIRECTIONAL = 9,
  MEDIAN = 10,
  SPECIAL1 = 11,
  SPECIAL2 = 12,
  SPECIAL3 = 13,
  ROAD_WORKS = 14,
  TRAM = 15,
  RAIL = 16,
  ENTRY = 17,
  EXIT = 18,
  OFF_RAMP = 19,
  ON_RAMP = 20,
  CONNECTING_RAMP = 21,
  BUS = 22,
  TAXI = 23,
  HOV = 24
  */
};

namespace roadmark {

enum XodrRoadMarkType {
  NONE = 0,
  SOLID = 1,
  BROKEN = 2,
  /*SOLID_SOLID = 3, // (for double solid line)
  SOLID_BROKEN = 4, // (from inside to outside, exception: center lane – from
  left to right) BROKEN_SOLID = 5, // (from inside to outside, exception: center
  lane – from left to right) BROKEN_BROKEN = 6, // (from inside to outside,
  exception: center lane – from left to right) BOTTS_DOTS = 7, GRASS = 8, //
  (meaning a grass edge) CURB = 9, CUSTOM = 10, //  (if detailed description is
  given in child tags) EDGE = 11, // (describing the limit of usable space on a
  road)
  */
};

enum XodrRoadMarkColor {
  STANDARD = 0,  // (equivalent to "white")
  /*BLUE = 1,
  GREEN = 2,
  RED = 3,
  */
  WHITE = 4,
  YELLOW = 5,
  // ORANGE = 6,
};

}  // namespace roadmark

struct XodrRoadMark {
  roadmark::XodrRoadMarkType type_;
  roadmark::XodrRoadMarkColor color_;
  float width_;
};

inline std::string print(const XodrRoadMark& r) {
  std::stringstream ss;
  ss << "XodrRoadMark: type: " << r.type_ << ", color: " << r.color_
     << ", width: " << r.width_ << std::endl;
  return ss.str();
}

struct XodrLaneWidth {
  float s_start;
  float s_end;
  XodrLaneOffset off;
};

inline geometry::Line CreateLineWithOffsetFromLine(
    geometry::Line previous_line, int id, XodrLaneWidth lane_width_current_lane,
    float s_inc = 0.2f, float s_max_delta = 0.1f) {
  namespace bg = boost::geometry;
  XodrLaneOffset off = lane_width_current_lane.off;
  float s = lane_width_current_lane.s_start;
  float s_end = lane_width_current_lane.s_end;
  float scale = 0.0f;

  boost::geometry::unique(previous_line.obj_);

  geometry::Line simplified_prev_line;
  boost::geometry::simplify(previous_line.obj_, simplified_prev_line.obj_,
                            s_max_delta);
  simplified_prev_line.RecomputeS();

  geometry::Line tmp_line;
  geometry::Point2d normal(0.0f, 0.0f);
  int sign = id > 0 ? -1 : 1;
  if (s_end > simplified_prev_line.Length())
    s_end = simplified_prev_line.Length();

  // b, c, d, s_start = 0 and roughly the same length simplification
  if (off.b == 0. && off.c == 0. && off.d == 0. &&
      simplified_prev_line.obj_.size() > 1 && s == 0. &&
      fabs((s_end - s) - simplified_prev_line.Length()) < 1.) {
    // we can loop through all innter lane points
    // previous_line == inner_line
    geometry::Point2d prev_point = simplified_prev_line.obj_[0],
                      current_point = simplified_prev_line.obj_[1];

    float tangent_angle =
        atan2(bg::get<1>(current_point) - bg::get<1>(prev_point),
              bg::get<0>(current_point) - bg::get<0>(prev_point));
    normal = geometry::Point2d(cos(tangent_angle + asin(1)),
                               sin(tangent_angle + asin(1)));
    scale = -sign * off.a;
    tmp_line.AddPoint(
        geometry::Point2d(bg::get<0>(prev_point) + scale * bg::get<0>(normal),
                          bg::get<1>(prev_point) + scale * bg::get<1>(normal)));

    for (int i = 1; i < simplified_prev_line.obj_.size(); i++) {
      prev_point = simplified_prev_line.obj_[i - 1];
      current_point = simplified_prev_line.obj_[i];
      tangent_angle = atan2(bg::get<1>(current_point) - bg::get<1>(prev_point),
                            bg::get<0>(current_point) - bg::get<0>(prev_point));
      normal = geometry::Point2d(cos(tangent_angle + asin(1)),
                                 sin(tangent_angle + asin(1)));
      tmp_line.AddPoint(geometry::Point2d(
          bg::get<0>(current_point) + scale * bg::get<0>(normal),
          bg::get<1>(current_point) + scale * bg::get<1>(normal)));
    }
  } else {
    for (; s <= s_end;) {
      geometry::Point2d point = GetPointAtS(simplified_prev_line, s);
      normal = GetNormalAtS(simplified_prev_line, s);
      scale = -sign * Polynom(s - lane_width_current_lane.s_start, off.a, off.b,
                              off.c, off.d);
      tmp_line.AddPoint(
          geometry::Point2d(bg::get<0>(point) + scale * bg::get<0>(normal),
                            bg::get<1>(point) + scale * bg::get<1>(normal)));
      if ((s_end - s < s_inc) && (s_end - s > 0.)) s_inc = s_end - s;
      s += s_inc;
    }
  }

  // SIMPLIFY line with max error
  geometry::Line simplified_line;
  boost::geometry::simplify(tmp_line.obj_, simplified_line.obj_, s_max_delta);
  simplified_line.RecomputeS();
  return simplified_line;
}

// using XodrLaneWidths = std::vector<XodrLaneWidth>;

}  // namespace opendrive
}  // namespace world
}  // namespace bark

#endif  // BARK_WORLD_OPENDRIVE_COMMONS_HPP_
