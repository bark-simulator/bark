// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/world/tests/make_test_xodr_map.hpp"
#include <vector>
#include "bark/commons/params/setter_params.hpp"
#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"

using bark::world::opendrive::OpenDriveMapPtr;

OpenDriveMapPtr bark::world::tests::MakeXodrMapOneRoadTwoLanes() {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;

  OpenDriveMapPtr open_drive_map = std::make_shared<OpenDriveMap>();

  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 200.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! Plan View
  XodrLanePtr lane0(new XodrLane(0));
  lane0->SetLine(p->GetReferenceLine());

  //! Lanes
  XodrLaneOffset off = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 200, off};

  XodrLanePtr lane1 =
      CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width_1, 0.5);
  lane1->SetLaneType(XodrLaneType::DRIVING);
  lane1->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  XodrLanePtr lane2 =
      CreateLaneFromLaneWidth(-2, lane1->GetLine(), lane_width_1, 0.5);
  lane2->SetLaneType(XodrLaneType::DRIVING);
  lane2->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  ls->AddLane(lane0);
  ls->AddLane(lane1);
  ls->AddLane(lane2);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  open_drive_map->AddRoad(r);

  return open_drive_map;
}

OpenDriveMapPtr bark::world::tests::MakeXodrMapTwoRoadsOneLane() {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;

  OpenDriveMapPtr open_drive_map = std::make_shared<OpenDriveMap>();

  // ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 50.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! Plan View
  XodrLanePtr lane00(new XodrLane(0));
  lane00->SetLine(p->GetReferenceLine());

  //! Lanes
  XodrLaneOffset off = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 50, off};

  XodrLanePtr lane10 =
      CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width_1, 0.5);
  lane10->SetLaneType(XodrLaneType::DRIVING);
  lane10->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  XodrLaneLink ll1;
  ll1.from_position = -1;
  ll1.to_position = -1;
  lane10->SetLink(ll1);

  ls->AddLane(lane00);
  ls->AddLane(lane10);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  //! ROAD 2
  //! Plan View
  PlanViewPtr p2(new PlanView());
  p2->AddLine(Point2d(50.0f, 0.0f), 0.0f, 100.0f);

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane01(new XodrLane(0));
  lane01->SetLine(p2->GetReferenceLine());

  XodrLaneOffset off2 = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 50, off};
  XodrLanePtr lane11 =
      CreateLaneFromLaneWidth(-1, p2->GetReferenceLine(), lane_width_2, 0.5);
  lane11->SetLaneType(XodrLaneType::DRIVING);
  lane11->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  XodrLaneLink ll2;
  ll2.from_position = -1;
  ll2.to_position = -1;
  lane11->SetLink(ll2);
  ls2->AddLane(lane01);
  ls2->AddLane(lane11);

  XodrRoadPtr r2(new XodrRoad("highway", 101));
  r2->SetPlanView(p2);
  r2->AddLaneSection(ls2);

  XodrRoadLinkInfo rli_succ(r2->GetId(), "road");
  XodrRoadLink rl;
  rl.SetSuccessor(rli_succ);
  r->SetLink(rl);

  open_drive_map->AddRoad(r);
  open_drive_map->AddRoad(r2);

  return open_drive_map;
}

OpenDriveMapPtr bark::world::tests::MakeXodrMapEndingLaneInParallel() {
  using namespace bark::geometry;
  using namespace bark::world::opendrive;

  OpenDriveMapPtr open_drive_map = std::make_shared<OpenDriveMap>();

  // ROAD 1
  PlanViewPtr p(new PlanView());
  p->AddLine(Point2d(0.0f, 0.0f), 0.0f, 50.0f);

  //! XodrLane-Section 1
  XodrLaneSectionPtr ls(new XodrLaneSection(0.0));

  //! Plan View
  XodrLanePtr lane00(new XodrLane(0));
  lane00->SetLine(p->GetReferenceLine());

  //! Lanes
  XodrLaneOffset off = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_1 = {0, 50, off};

  XodrLanePtr lane10 =
      CreateLaneFromLaneWidth(-1, p->GetReferenceLine(), lane_width_1, 0.5);
  lane10->SetLaneType(XodrLaneType::DRIVING);
  lane10->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  XodrLanePtr lane20 =
      CreateLaneFromLaneWidth(-2, lane10->GetLine(), lane_width_1, 0.5);
  lane20->SetLaneType(XodrLaneType::DRIVING);
  lane20->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  XodrLaneLink ll1;
  ll1.from_position = -1;
  ll1.to_position = -1;
  lane10->SetLink(ll1);

  ls->AddLane(lane00);
  ls->AddLane(lane10);
  ls->AddLane(lane20);

  XodrRoadPtr r(new XodrRoad("highway", 100));
  r->SetPlanView(p);
  r->AddLaneSection(ls);

  //! ROAD 2
  //! Plan View
  PlanViewPtr p2(new PlanView());
  p2->AddLine(Point2d(50.0f, 0.0f), 0.0f, 100.0f);

  //! XodrLane-Section 2
  XodrLaneSectionPtr ls2(new XodrLaneSection(0.0));

  //! XodrLane
  XodrLanePtr lane01(new XodrLane(0));
  lane01->SetLine(p2->GetReferenceLine());

  XodrLaneOffset off2 = {3.5f, 0.0f, 0.0f, 0.0f};
  XodrLaneWidth lane_width_2 = {0, 50, off};
  XodrLanePtr lane11 =
      CreateLaneFromLaneWidth(-1, p2->GetReferenceLine(), lane_width_2, 0.5);
  lane11->SetLaneType(XodrLaneType::DRIVING);
  lane11->SetDrivingDirection(XodrDrivingDirection::FORWARD);

  XodrLaneLink ll2;
  ll2.from_position = -1;
  ll2.to_position = -1;
  lane11->SetLink(ll2);
  ls2->AddLane(lane01);
  ls2->AddLane(lane11);

  XodrRoadPtr r2(new XodrRoad("highway", 101));
  r2->SetPlanView(p2);
  r2->AddLaneSection(ls2);

  XodrRoadLinkInfo rli_succ(r2->GetId(), "road");
  XodrRoadLink rl;
  rl.SetSuccessor(rli_succ);
  r->SetLink(rl);

  open_drive_map->AddRoad(r);
  open_drive_map->AddRoad(r2);

  return open_drive_map;
}