// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "modules/world/tests/make_test_xodr_map.hpp"
#include <vector>
#include "modules/commons/params/default_params.hpp"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"

using modules::world::opendrive::OpenDriveMapPtr;

OpenDriveMapPtr modules::world::tests::make_xodr_map_one_road_two_lanes() {
  using namespace modules::geometry;
  using namespace modules::world::opendrive;

  OpenDriveMapPtr open_drive_map = std::make_shared<OpenDriveMap>();

  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 200.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! Plan View
  XodrLanePtr lane0(new XodrLane(0));
  lane0->set_line(p->get_reference_line());

  //! Lanes
  XodrLaneOffset off = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 200, off};

  XodrLanePtr lane1 = create_lane_from_lane_width(-1, p->get_reference_line(),
                                                  lane_width_1, 0.5);
  lane1->set_lane_type(XodrLaneType::DRIVING);
  lane1->set_driving_direction(XodrDrivingDirection::FORWARD);

  XodrLanePtr lane2 =
      create_lane_from_lane_width(-2, lane1->get_line(), lane_width_1, 0.5);
  lane2->set_lane_type(XodrLaneType::DRIVING);
  lane2->set_driving_direction(XodrDrivingDirection::FORWARD);

  ls->add_lane(lane0);
  ls->add_lane(lane1);
  ls->add_lane(lane2);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  open_drive_map->add_road(r);

  return open_drive_map;
}

OpenDriveMapPtr modules::world::tests::make_xodr_map_two_roads_one_lane() {
  using namespace modules::geometry;
  using namespace modules::world::opendrive;

  OpenDriveMapPtr open_drive_map = std::make_shared<OpenDriveMap>();

  // ROAD 1
  PlanViewPtr p(new PlanView());
  p->add_line(Point2d(0.0f, 0.0f), 0.0f, 50.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! Plan View
  XodrLanePtr lane00(new XodrLane(0));
  lane00->set_line(p->get_reference_line());

  //! Lanes
  XodrLaneOffset off = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 50, off};

  XodrLanePtr lane10 = create_lane_from_lane_width(-1, p->get_reference_line(),
                                                   lane_width_1, 0.5);
  lane10->set_lane_type(XodrLaneType::DRIVING);
  lane10->set_driving_direction(XodrDrivingDirection::FORWARD);

  XodrLaneLink ll1;
  ll1.from_position = -1;
  ll1.to_position = -1;
  lane10->set_link(ll1);

  ls->add_lane(lane00);
  ls->add_lane(lane10);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->set_plan_view(p);
  r->add_lane_section(ls);

  //! ROAD 2
  //! Plan View
  PlanViewPtr p2(new PlanView());
  p2->add_line(Point2d(50.0f, 0.0f), 0.0f, 100.0f);

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane01(new XodrLane(0));
  lane01->set_line(p2->get_reference_line());

  XodrLaneOffset off2 = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 50, off};
  XodrLanePtr lane11 = create_lane_from_lane_width(-1, p2->get_reference_line(),
                                                   lane_width_2, 0.05);
  lane11->set_lane_type(XodrLaneType::DRIVING);
  lane11->set_driving_direction(XodrDrivingDirection::FORWARD);

  XodrLaneLink ll2;
  ll2.from_position = -1;
  ll2.to_position = -1;
  lane11->set_link(ll2);
  ls2->add_lane(lane01);
  ls2->add_lane(lane11);

  XodrRoadPtr r2(new XodrRoad("highway", 101));
  r2->set_plan_view(p2);
  r2->add_lane_section(ls2);

  XodrRoadLinkInfo rli_succ(r2->get_id(), "road");
  XodrRoadLink rl;
  rl.set_successor(rli_succ);
  r->set_link(rl);

  open_drive_map->add_road(r);
  open_drive_map->add_road(r2);

  return open_drive_map;
}