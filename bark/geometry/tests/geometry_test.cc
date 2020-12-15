// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "bark/geometry/commons.hpp"
#include "bark/geometry/line.hpp"
#include "bark/geometry/polygon.hpp"
#include "bark/geometry/standard_shapes.hpp"
#include "gtest/gtest.h"

TEST(polygon, base_functionality) {
  using bark::geometry::Point2d;
  using bark::geometry::Point2d_t;
  using bark::geometry::Polygon;
  using bark::geometry::Polygon_t;

  // template version; point <--> polygon
  Point2d_t<double> point_1(0.0, 0.0);
  Point2d_t<double> point_2(4.0, 0.0);
  Point2d_t<double> point_3(4.0, 4.0);
  Point2d_t<double> point_4(0.0, 4.0);

  Polygon_t<Point2d_t<double>> polygon;
  polygon.AddPoint(point_1);
  polygon.AddPoint(point_2);
  polygon.AddPoint(point_3);
  polygon.AddPoint(point_4);

  Point2d_t<double> check_point(5.0, 2.0);

  EXPECT_NEAR(Distance(polygon, check_point), 1.0, 0.01);

  // polygon <--> polygon
  Point2d pointf_1(5.0, 2.0);
  Point2d pointf_2(7.5, 0.0);
  Point2d pointf_3(9.0, 2.0);
  Point2d pointf_4(7.5, 4.0);

  Polygon polygon_2;
  polygon_2.AddPoint(pointf_1);
  polygon_2.AddPoint(pointf_2);
  polygon_2.AddPoint(pointf_3);
  polygon_2.AddPoint(pointf_4);

  EXPECT_NEAR(Distance(polygon, polygon_2), 1.0, 0.01);
}

TEST(line, base_functionality) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  using bark::geometry::Point2d_t;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);

  Point2d_t<double> check_point(2.5, 2.0);

  EXPECT_NEAR(Distance(line, check_point), 2.5, 0.01);
}

TEST(geometry, line) {
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;
  namespace bg = boost::geometry;

  Line l;  // vertical
  l.AddPoint(Point2d(0.0, 0.0));
  l.AddPoint(Point2d(0.0, 10.0));

  Point2d p = GetPointAtS(l, 5.0);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 5.0, 0.1);

  p = GetNormalAtS(l, 5.0);
  EXPECT_NEAR(bg::get<0>(p), -1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = GetNearestPoint(l, Point2d(5.0, 5.0));
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 5.0, 0.1);

  Line l2;  // horizontal
  l2.AddPoint(Point2d(0.0, 0.0));
  l2.AddPoint(Point2d(10.0, 0.0));

  p = GetPointAtS(l2, 5.0);
  EXPECT_NEAR(bg::get<0>(p), 5.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = GetNormalAtS(l2, 5.0);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 1.0, 0.1);

  p = GetNearestPoint(l2, Point2d(5.0, 5.0));
  EXPECT_NEAR(bg::get<0>(p), 5.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  Line l3;  // 45 deg
  l3.AddPoint(Point2d(0.0, 0.0));
  l3.AddPoint(Point2d(10.0, 10.0));

  p = GetPointAtS(l3, 0.5 * sqrt(200));
  EXPECT_NEAR(bg::get<0>(p), 5.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 5.0, 0.1);

  p = GetNormalAtS(l3, 0.5 * sqrt(200));
  EXPECT_NEAR(bg::get<0>(p), -0.7, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.7, 0.1);

  Line l4;  // 45 deg
  l4.AddPoint(Point2d(0.0, 0.0));
  l4.AddPoint(Point2d(-10.0, 10.0));

  p = GetNormalAtS(l4, 0.5 * sqrt(200));
  EXPECT_NEAR(bg::get<0>(p), -0.7, 0.1);
  EXPECT_NEAR(bg::get<1>(p), -0.7, 0.1);

  EXPECT_NEAR(GetNearestS(l, Point2d(5, 5)), 5.0, 0.1);
  EXPECT_NEAR(GetNearestS(l3, Point2d(5, 5)), 0.5 * sqrt(200), 0.1);
}

TEST(geometry, line_transform) {
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  Line line_in;  // vertical
  line_in.AddPoint(Point2d(0.0, 0.0));
  line_in.AddPoint(Point2d(0.0, 10.0));

  double hdg = 3.14159265359;
  double offset_x = 1;
  double offset_y = 2;

  Line obj_rotated = Rotate(line_in, hdg);
  EXPECT_NEAR(line_in.Length(), obj_rotated.Length(), 0.01);

  EXPECT_NEAR(bg::get<0>(obj_rotated.obj_.at(0)), 0, 0);
  EXPECT_NEAR(bg::get<1>(obj_rotated.obj_.at(0)), 0, 0);
  EXPECT_NEAR(bg::get<0>(obj_rotated.obj_.at(1)), 0, 0.1);
  EXPECT_NEAR(bg::get<1>(obj_rotated.obj_.at(1)), -10, 0.1);

  Line obj_transformed = Translate(obj_rotated, offset_x, offset_y);
  EXPECT_NEAR(line_in.Length(), obj_transformed.Length(), 0.01);

  EXPECT_NEAR(bg::get<0>(obj_transformed.obj_.at(0)), 1, 0);
  EXPECT_NEAR(bg::get<1>(obj_transformed.obj_.at(0)), 2, 0);
  EXPECT_NEAR(bg::get<0>(obj_transformed.obj_.at(1)), 1, 0.1);
  EXPECT_NEAR(bg::get<1>(obj_transformed.obj_.at(1)), -8, 0.1);
}

TEST(geometry, polygon) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p;
  p.AddPoint(Point2d(0, 0));
  p.AddPoint(Point2d(0, 2));
  p.AddPoint(Point2d(2, 4));
  p.AddPoint(Point2d(4, 0));
  p.AddPoint(Point2d(0, 0));

  EXPECT_TRUE(p.Valid());

  std::shared_ptr<Polygon> p2 =
      std::dynamic_pointer_cast<Polygon>(p.Rotate(3.14 / 2));
}

TEST(geometry, standard_shapes) {
  using bark::geometry::Polygon;
  using bark::geometry::standard_shapes::CarLimousine;
  using bark::geometry::standard_shapes::CarRectangle;

  Polygon p1 = CarLimousine();
  ASSERT_TRUE(p1.Valid());

  Polygon p2 = CarRectangle();
  ASSERT_TRUE(p2.Valid());
}

// poly point collide false
TEST(collision, poly2point1) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p;
  p.AddPoint(Point2d(0, 0));
  p.AddPoint(Point2d(0, 2));
  p.AddPoint(Point2d(2, 4));
  p.AddPoint(Point2d(4, 0));
  p.AddPoint(Point2d(0, 0));

  Point2d pt(-2, -2);

  EXPECT_FALSE(Collide(p, pt));
}

// poly point collide true
TEST(collision, poly2point2) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p;
  p.AddPoint(Point2d(0, 0));
  p.AddPoint(Point2d(0, 2));
  p.AddPoint(Point2d(2, 4));
  p.AddPoint(Point2d(4, 0));
  p.AddPoint(Point2d(0, 0));

  Point2d pt(1, 1);

  EXPECT_TRUE(Collide(p, pt));
}

// poly poly collide false
TEST(collision, poly2poly1) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Polygon p2;
  p2.AddPoint(Point2d(10, 10));
  p2.AddPoint(Point2d(10, 11));
  p2.AddPoint(Point2d(11, 11));
  p2.AddPoint(Point2d(11, 10));
  p2.AddPoint(Point2d(10, 10));

  EXPECT_FALSE(Collide(p1, p2));

  EXPECT_FALSE(Collide(p2, p1));
}

// poly poly collide overlap true
TEST(collision, poly2poly2) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Polygon p2;
  p2.AddPoint(Point2d(0.5, 0.5));
  p2.AddPoint(Point2d(0.5, 1.5));
  p2.AddPoint(Point2d(1.5, 1.5));
  p2.AddPoint(Point2d(1.5, 0.5));
  p2.AddPoint(Point2d(0.5, 0.5));

  EXPECT_TRUE(Collide(p1, p2));

  EXPECT_TRUE(Collide(p2, p1));
}

// poly poly collide point overlap true
TEST(collision, poly2poly3) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Polygon p2;
  p2.AddPoint(Point2d(1, 1));
  p2.AddPoint(Point2d(1, 2));
  p2.AddPoint(Point2d(2, 2));
  p2.AddPoint(Point2d(2, 1));
  p2.AddPoint(Point2d(1, 1));

  EXPECT_TRUE(Collide(p1, p2));

  EXPECT_TRUE(Collide(p2, p1));
}

// poly poly collide edge overlap true
TEST(collision, poly2poly4) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Polygon p2;
  p2.AddPoint(Point2d(1, 1));
  p2.AddPoint(Point2d(2, 1));
  p2.AddPoint(Point2d(2, 0));
  p2.AddPoint(Point2d(1, 0));
  p2.AddPoint(Point2d(1, 1));

  EXPECT_TRUE(Collide(p1, p2));

  EXPECT_TRUE(Collide(p2, p1));
}

// poly poly collide ccw true
TEST(collision, poly2poly5) {
  using bark::geometry::Collide;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Polygon p2;
  p2.AddPoint(Point2d(1.5, 1.5));
  p2.AddPoint(Point2d(0.5, 1.5));
  p2.AddPoint(Point2d(0.5, 0.5));
  p2.AddPoint(Point2d(1.5, 0.5));
  p2.AddPoint(Point2d(1.5, 1.5));

  EXPECT_TRUE(Collide(p1, p2));

  EXPECT_TRUE(Collide(p2, p1));
}

// poly line collision no collision
TEST(collision, poly2line1) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Line l1;
  l1.AddPoint(Point2d(1.5, 1.5));
  l1.AddPoint(Point2d(2, 2));

  EXPECT_FALSE(Collide(p1, l1));
}

// poly line collision line intersect
TEST(collision, poly2line2) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Line l1;
  l1.AddPoint(Point2d(0.5, 0.5));
  l1.AddPoint(Point2d(2, 2));

  EXPECT_TRUE(Collide(p1, l1));

  Line l2;
  l2.AddPoint(Point2d(-0.5, -0.5));
  l2.AddPoint(Point2d(2, 2));

  EXPECT_TRUE(Collide(p1, l2));
}

// poly line collision point intersect
TEST(collision, poly2line3) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon p1;
  p1.AddPoint(Point2d(0, 0));
  p1.AddPoint(Point2d(0, 1));
  p1.AddPoint(Point2d(1, 1));
  p1.AddPoint(Point2d(1, 0));
  p1.AddPoint(Point2d(0, 0));

  Line l1;
  l1.AddPoint(Point2d(1, 1));
  l1.AddPoint(Point2d(2, 2));

  EXPECT_TRUE(Collide(p1, l1));
}

// line point collision no intersect
TEST(collision, line2point1) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;

  Line l1;
  l1.AddPoint(Point2d(1, 1));
  l1.AddPoint(Point2d(2, 2));

  Point2d p(0, 0);

  EXPECT_FALSE(Collide(l1, p));
}

// line point collision point intersect
TEST(collision, line2point2) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;

  Line l1;
  l1.AddPoint(Point2d(0, 0));
  l1.AddPoint(Point2d(2, 2));

  Point2d p(0.5, 0.5);

  EXPECT_TRUE(Collide(l1, p));
}

// line point collision end point intersect
TEST(collision, line2point3) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;

  Line l1;
  l1.AddPoint(Point2d(1, 1));
  l1.AddPoint(Point2d(2, 2));

  Point2d p(1, 1);

  EXPECT_TRUE(Collide(l1, p));
}

// line line collision no intersect
TEST(collision, line2line1) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;

  Line l1;
  l1.AddPoint(Point2d(1, 1));
  l1.AddPoint(Point2d(2, 2));

  Line l2;
  l2.AddPoint(Point2d(3, 3));
  l2.AddPoint(Point2d(4, 4));

  EXPECT_FALSE(Collide(l1, l2));
}

// line line collision point intersect
TEST(collision, line2line2) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  Line l1;
  l1.AddPoint(Point2d(1, 1));
  l1.AddPoint(Point2d(2, 2));

  Line l2;
  l2.AddPoint(Point2d(1, 1));
  l2.AddPoint(Point2d(4, 4));

  EXPECT_TRUE(Collide(l1, l2));
  EXPECT_TRUE(Collide(l2, l1));
}

// line line collision point intersect point
TEST(collision, line2line3) {
  using bark::geometry::Collide;
  using bark::geometry::Line;
  using bark::geometry::Point2d;

  Line l1;
  l1.AddPoint(Point2d(1, 1));
  l1.AddPoint(Point2d(2, 2));

  Line l2;
  l2.AddPoint(Point2d(2, 2));
  l2.AddPoint(Point2d(4, 4));

  EXPECT_TRUE(Collide(l1, l2));
  EXPECT_TRUE(Collide(l2, l1));
}

// car shape collision false
TEST(collision, carshape1) {
  using bark::geometry::Collide;
  using bark::geometry::Polygon;
  using bark::geometry::Pose;
  using bark::geometry::standard_shapes::CarLimousine;

  Polygon outline = CarLimousine();
  std::shared_ptr<Polygon> car1 =
      std::dynamic_pointer_cast<Polygon>(outline.Transform(Pose(0, 0, 0)));
  std::shared_ptr<Polygon> car2 =
      std::dynamic_pointer_cast<Polygon>(outline.Transform(Pose(10, 10, 0)));

  EXPECT_FALSE(Collide(*car1, *car2));
}

// car shape collision true
TEST(collision, carshape2) {
  using bark::geometry::Collide;
  using bark::geometry::Polygon;
  using bark::geometry::Pose;
  using bark::geometry::standard_shapes::CarLimousine;

  Polygon outline = CarLimousine();
  std::shared_ptr<Polygon> car1 =
      std::dynamic_pointer_cast<Polygon>(outline.Transform(Pose(0, 0, 0)));
  std::shared_ptr<Polygon> car2 =
      std::dynamic_pointer_cast<Polygon>(outline.Transform(Pose(1, 0, 3.14)));

  // TODO(@hart): HACK
  // EXPECT_TRUE(Collide(car1.get(), car2.get()));
}

TEST(line, s1) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);

  EXPECT_DOUBLE_EQ(line.s_[0], 0.0);
  EXPECT_DOUBLE_EQ(line.s_[1], 1.0);
  EXPECT_DOUBLE_EQ(line.s_[2], 2.0);
}

TEST(line, s2) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;

  // template
  Point2d point_1(-1.0, 0.0);
  Point2d point_2(0.0, 0.0);
  Point2d point_3(1.0, 0.0);
  Point2d point_4(3.0, 0.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);
  line.AddPoint(point_4);

  EXPECT_EQ(line.s_.size(), 4u);
  EXPECT_DOUBLE_EQ(line.s_[0], 0.0);
  EXPECT_DOUBLE_EQ(line.s_[1], 1.0);
  EXPECT_DOUBLE_EQ(line.s_[2], 2.0);
  EXPECT_DOUBLE_EQ(line.s_[3], 4.0);
}

TEST(line, GetS_at_pt_1) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  using bark::geometry::operator==;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);

  Point2d p1 = GetPointAtS(line, 0.0);
  Point2d p2 = GetPointAtS(line, 1.0);
  Point2d p3 = GetPointAtS(line, 2.0);

  Point2d p4 = GetPointAtS(line, 0.5);
  Point2d p5 = GetPointAtS(line, 1.5);

  EXPECT_TRUE(point_1 == p1);
  EXPECT_TRUE(point_2 == p2);
  EXPECT_TRUE(point_3 == p3);

  EXPECT_TRUE(Point2d(0.0, 1.5) == p4);
  EXPECT_TRUE(Point2d(0.0, 2.5) == p5);
}

TEST(line, GetPtAtIdx) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  using bark::geometry::operator==;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);

  Point2d p1 = GetPointAtIdx(line, 0);
  Point2d p2 = GetPointAtIdx(line, 1);
  Point2d p3 = GetPointAtIdx(line, 2);

  EXPECT_TRUE(point_1 == p1);
  EXPECT_TRUE(point_2 == p2);
  EXPECT_TRUE(point_3 == p3);
}

TEST(line, GetLineFromSInterval) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  using bark::geometry::operator==;

  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);

  Line_t<Point2d> line_segment = GetLineFromSInterval(line, 0.5, 1.5);

  Point2d p1 = GetPointAtS(line_segment, 0.0);
  Point2d p2 = GetPointAtS(line_segment, 0.5);
  Point2d p3 = GetPointAtS(line_segment, 0.9);
  Point2d p4 = GetPointAtS(line_segment, 1.0);

  EXPECT_TRUE(Point2d(0.0, 1.5) == p1);
  EXPECT_TRUE(point_2 == p2);
  
  EXPECT_NEAR(boost::geometry::get<0>(p3), 0.0, 1e-3);
  EXPECT_NEAR(boost::geometry::get<1>(p3), 2.4, 1e-3);

  EXPECT_TRUE(Point2d(0.0, 2.5) == p4);
}
TEST(line, GetLineFromSInterval_entire_line) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  using bark::geometry::operator==;

  Point2d point_1(0.0, 0.0);
  Point2d point_2(0.0, 2.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);

  Line_t<Point2d> line_segment = GetLineFromSInterval(line, 0.0, 2.0);

  EXPECT_TRUE(Point2d(0.0, 0.0) == point_1);
  EXPECT_TRUE(Point2d(0.0, 2.0) == point_2);
}

TEST(line, GetNearestPoint_1) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  using bark::geometry::operator==;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);

  Point2d p1 = GetNearestPoint(line, point_1);
  Point2d p2 = GetNearestPoint(line, point_2);
  Point2d p3 = GetNearestPoint(line, point_3);

  Point2d p4 = GetNearestPoint(line, Point2d(0.0, 0.0));
  Point2d p5 = GetNearestPoint(line, Point2d(0.0, 4.0));

  Point2d p6 = GetNearestPoint(line, Point2d(1.0, 1.0));
  Point2d p7 = GetNearestPoint(line, Point2d(1.0, 2.2));

  EXPECT_TRUE(point_1 == p1);
  EXPECT_TRUE(point_2 == p2);
  EXPECT_TRUE(point_3 == p3);

  EXPECT_TRUE(Point2d(0.0, 1.0) == p4);
  EXPECT_TRUE(Point2d(0.0, 3.0) == p5);

  EXPECT_TRUE(Point2d(0.0, 1.0) == p6);
  EXPECT_NEAR(boost::geometry::get<0>(p7), 0, 0.0001);
  EXPECT_NEAR(boost::geometry::get<1>(p7), 2.2, 0.0001);
}

TEST(line, segment_intersection_check_1) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(0.0, 1.0);
  Point2d point_3(0.0, 2.0);
  Point2d point_4(0.0, 3.0);
  Point2d point_5(0.0, 4.0);
  Point2d point_6(0.0, 5.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);
  line.AddPoint(point_4);
  line.AddPoint(point_5);
  line.AddPoint(point_6);

  EXPECT_NEAR(GetSegmentEndIdx(line, 0.0), 1, 0.1);
  EXPECT_NEAR(GetSegmentEndIdx(line, 3.0), 4, 0.1);
  EXPECT_NEAR(GetSegmentEndIdx(line, 6.0), 5, 0.1);
}
TEST(line, segment_intersection_tangent_1) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;

  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(1.0, 1.0);        // s = sqrt(2)
  Point2d point_3(2.0, 0.0);        // 2 * sqrt(2)
  Point2d point_4(2.0, sqrt(2.0));  // 3* sqrt(2)
  Point2d point_5(2.0 + sqrt(2.0), sqrt(2.0));

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);
  line.AddPoint(point_4);
  line.AddPoint(point_5);

  EXPECT_NEAR(GetTangentAngleAtS(line, sqrt(2)), 0, 0.1);
  EXPECT_NEAR(GetTangentAngleAtS(line, 2 * sqrt(2)), (1.0 / 8.0) * 3.141, 0.1);

  // template
  Point2d point_6(0.0, 0.0);
  Point2d point_7(0, 1.0);
  Point2d point_8(0.0, 2.0);
  Point2d point_9(0, 3.0);
  Point2d point_10(0.0, 4.0);

  Line_t<Point2d> line2;

  line2.AddPoint(point_6);
  line2.AddPoint(point_7);
  line2.AddPoint(point_8);
  line2.AddPoint(point_9);
  line2.AddPoint(point_10);

  EXPECT_NEAR(GetTangentAngleAtS(line2, 0.25), (1.0 / 2.0) * 3.141, 0.1);
  EXPECT_NEAR(GetTangentAngleAtS(line2, 0.5), (1.0 / 2.0) * 3.141, 0.1);
}

TEST(line, segment_get_normal_1) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(1.0, 0.0);
  Point2d point_3(1.0, 1.0);
  Point2d point_4(0.0, 1.0);

  Line_t<Point2d> line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);
  line.AddPoint(point_4);
  line.AddPoint(point_1);

  Point2d p = GetNormalAtS(line, 0.5);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 1.0, 0.1);

  p = GetNormalAtS(line, 1.5);
  EXPECT_NEAR(bg::get<0>(p), -1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = GetNormalAtS(line, 2.5);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), -1.0, 0.1);

  p = GetNormalAtS(line, 3.5);
  EXPECT_NEAR(bg::get<0>(p), 1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = GetNormalAtS(line, 4.0);
  EXPECT_NEAR(bg::get<0>(p), 1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);
}

TEST(line, gradient) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  Line_t<Point2d> line;
  line.AddPoint(Point2d(0.0, 0.0));
  line.AddPoint(Point2d(0.2, 0.3090));
  line.AddPoint(Point2d(0.4, 0.5878));
  line.AddPoint(Point2d(0.6, 0.8090));
  line.AddPoint(Point2d(0.8, 0.9511));
  line.AddPoint(Point2d(1.0, 1.0000));

  Eigen::VectorXd gradX = bark::geometry::Gradient(line.ToArray().col(0));
  Eigen::VectorXd gradY = bark::geometry::Gradient(line.ToArray().col(1));

  Eigen::VectorXd gradX_expect(6);
  gradX_expect << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
  Eigen::VectorXd gradY_expect(6);
  gradY_expect << 0.3090, 0.2939, 0.2500, 0.1816, 0.0955, 0.0489;

  // do not compare first and last values
  double precision = 1e-3;
  EXPECT_TRUE(gradX.size() == gradX_expect.size()) << gradX.size() << std::endl
                                                   << gradX_expect.size();
  EXPECT_TRUE(
      gradX.segment(1, 4).isApprox(gradX_expect.segment(1, 4), precision))
      << gradX << gradX_expect;

  EXPECT_TRUE(gradY.size() == gradY_expect.size()) << gradY.size() << std::endl
                                                   << gradY_expect.size();
  EXPECT_TRUE(
      gradY.segment(1, 4).isApprox(gradY_expect.segment(1, 4), precision))
      << gradY << gradY_expect;
}

TEST(line, curvature) {
  using bark::geometry::Line_t;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  Line_t<Point2d> line;
  line.AddPoint(Point2d(0.0, 0.0));
  line.AddPoint(Point2d(0.2, 0.3090));
  line.AddPoint(Point2d(0.4, 0.5878));
  line.AddPoint(Point2d(0.5, 0.7071));
  line.AddPoint(Point2d(0.6, 0.8090));
  line.AddPoint(Point2d(0.8, 0.9511));
  line.AddPoint(Point2d(1.0, 1.0000));

  Eigen::VectorXd c = bark::geometry::GetCurvature(line);

  Eigen::VectorXd c_expect(7);
  c_expect << -0.0607, -0.0812, -0.2450, -1.1622, -1.0009, -0.8902, -1.0665;

  // do not compare first and last values, as gradient calculation is not
  // correct there
  double precision = 1e-3;
  EXPECT_TRUE(c.size() == c_expect.size()) << c.size() << std::endl
                                           << c_expect.size();
  EXPECT_NEAR(c(2), c_expect(2), precision);
  EXPECT_NEAR(c(3), c_expect(3), precision);
  EXPECT_NEAR(c(4), c_expect(4), precision);
}

TEST(optimizer, shrink_polygon) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;

  Polygon polygon;
  polygon.AddPoint(Point2d(0, 0));
  polygon.AddPoint(Point2d(4, 0));
  polygon.AddPoint(Point2d(4, 2));
  polygon.AddPoint(Point2d(6, 2));
  polygon.AddPoint(Point2d(6, 0));
  polygon.AddPoint(Point2d(10, 0));
  polygon.AddPoint(Point2d(10, 4));
  polygon.AddPoint(Point2d(0, 4));
  polygon.AddPoint(Point2d(0, 0));

  Polygon shrunk_polygon;
  BufferPolygon(polygon, -1, &shrunk_polygon);

  Polygon expected_shrunk_polygon;
  polygon.AddPoint(Point2d(1, 1));
  polygon.AddPoint(Point2d(3, 1));
  polygon.AddPoint(Point2d(3, 3));
  polygon.AddPoint(Point2d(7, 3));
  polygon.AddPoint(Point2d(7, 1));
  polygon.AddPoint(Point2d(7, 1));
  polygon.AddPoint(Point2d(9, 3));
  polygon.AddPoint(Point2d(9, 3));
  polygon.AddPoint(Point2d(1, 1));

  ASSERT_TRUE(Equals(expected_shrunk_polygon, shrunk_polygon));
}

TEST(line, line_smoothing) {
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(3.0, 1.0);
  Point2d point_3(4.0, 4.0);
  Point2d point_4(7.0, 5.0);

  Line line;

  line.AddPoint(point_1);
  line.AddPoint(point_2);
  line.AddPoint(point_3);
  line.AddPoint(point_4);

  double ds = 1.0;
  Line lss = bark::geometry::SmoothLine(line, ds);

  EXPECT_EQ(lss.size(), 11);
  EXPECT_TRUE(lss.Length() > line.Length());

  double tol = 0.3;
  EXPECT_EQ(bg::get<0>(lss.obj_[0]), 0.0);
  EXPECT_EQ(bg::get<1>(lss.obj_[0]), 0.0);

  EXPECT_NEAR(bg::get<0>(lss.obj_[1]), 1.4076, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[1]), -0.1427, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[2]), 2.3418, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[2]), 0.1881, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[3]), 2.9289, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[3]), 0.8659, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[4]), 3.2954, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[4]), 1.7642, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[5]), 3.5680, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[5]), 2.7566, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[6]), 3.8730, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[6]), 3.7165, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[7]), 4.3369, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[7]), 4.5175, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[8]), 5.0863, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[8]), 5.0330, tol);

  EXPECT_NEAR(bg::get<0>(lss.obj_[9]), 6.2476, tol);
  EXPECT_NEAR(bg::get<1>(lss.obj_[9]), 5.1366, tol);

  EXPECT_EQ(bg::get<0>(lss.obj_[10]), 7.0);
  EXPECT_EQ(bg::get<1>(lss.obj_[10]), 5.0);
}

TEST(line, append_line_no_intersect1) {
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  Line line1;
  line1.AddPoint(Point2d(0.0, 0.0));
  line1.AddPoint(Point2d(10.0, 0.0));

  Line line2;
  line2.AddPoint(Point2d(9.5, 0.5));
  line2.AddPoint(Point2d(13.0, -15.0));

  bool b = bg::intersects(line1.obj_, line2.obj_);
  EXPECT_TRUE(b);

  Line lout = AppendLinesNoIntersect(line1, line2);
  EXPECT_FALSE(bg::intersects(lout.obj_)) << lout.ToArray();
}


TEST(line, append_line_no_intersect2) {

  using bark::geometry::Line;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;

  Line line1;
  
  line1.AddPoint(Point2d(106.424, 103.972));
  line1.AddPoint(Point2d(101.24, 104.38));
  line1.AddPoint(Point2d(99.2392, 104.532));
  line1.AddPoint(Point2d(94.4502, 104.884));
  line1.AddPoint(Point2d(80.4863, 105.889));
  line1.AddPoint(Point2d(79.8727, 105.931));
  line1.AddPoint(Point2d(79.6731, 105.943));
  line1.AddPoint(Point2d(79.4609, 105.956));
  line1.AddPoint(Point2d(78.8466, 105.988));
  line1.AddPoint(Point2d(78.2345, 106.012));
  line1.AddPoint(Point2d(77.8347, 106.025));
  line1.AddPoint(Point2d(77.4247, 106.035));
  line1.AddPoint(Point2d(77.0147, 106.042));
  line1.AddPoint(Point2d(76.6047, 106.046));
  line1.AddPoint(Point2d(76.1946, 106.047));
  line1.AddPoint(Point2d(75.5733, 106.044));
  line1.AddPoint(Point2d(74.7734, 106.033));
  line1.AddPoint(Point2d(73.7679, 106.015));
  line1.AddPoint(Point2d(72.9625, 105.998));
  line1.AddPoint(Point2d(72.1578, 105.978));
  line1.AddPoint(Point2d(70.9488, 105.942));
  line1.AddPoint(Point2d(69.9493, 105.91));
  line1.AddPoint(Point2d(50.7593, 105.29));
  line1.AddPoint(Point2d(49.9589, 105.268));
  line1.AddPoint(Point2d(48.9649, 105.245));
  line1.AddPoint(Point2d(47.9938, 105.227));
  line1.AddPoint(Point2d(46.9931, 105.214));
  line1.AddPoint(Point2d(46.1989, 105.208));
  line1.AddPoint(Point2d(45.4046, 105.205));
  line1.AddPoint(Point2d(44.6104, 105.206));
  line1.AddPoint(Point2d(43.8161, 105.21));
  line1.AddPoint(Point2d(43.0218, 105.217));
  line1.AddPoint(Point2d(42.2276, 105.228));
  line1.AddPoint(Point2d(41.4342, 105.242));
  line1.AddPoint(Point2d(40.4409, 105.264));
  line1.AddPoint(Point2d(39.6469, 105.286));
  line1.AddPoint(Point2d(38.8534, 105.311));
  line1.AddPoint(Point2d(36.8547, 105.383));
  line1.AddPoint(Point2d(36.0579, 105.413));
  line1.AddPoint(Point2d(35.0637, 105.454));
  line1.AddPoint(Point2d(34.2695, 105.489));
  line1.AddPoint(Point2d(33.4748, 105.528));
  line1.AddPoint(Point2d(32.4754, 105.58));

  Line line2;
  line2.AddPoint(Point2d(32.4847, 105.579));
  line2.AddPoint(Point2d(31.4972, 105.635));
  line2.AddPoint(Point2d(30.5044, 105.695));
  line2.AddPoint(Point2d(29.5065, 105.759));
  line2.AddPoint(Point2d(28.5148, 105.827));
  line2.AddPoint(Point2d(27.3224, 105.914));
  line2.AddPoint(Point2d(22.3407, 106.294));
  line2.AddPoint(Point2d(10.7769, 107.211));
  line2.AddPoint(Point2d(0.608936, 108.018));
  line2.AddPoint(Point2d(-8.96079, 108.78));
  line2.AddPoint(Point2d(-18.1085, 109.51));

  Line lout = AppendLinesNoIntersect(line1, line2);
  EXPECT_FALSE(bg::intersects(lout.obj_)) << lout.ToArray();
}

TEST(line, concatenate_line_string) {
  using bark::geometry::Line;
  using bark::geometry::Point2d;
  namespace bg = boost::geometry;
  namespace barkg = bark::geometry;

  Line line1;
  line1.AddPoint(Point2d(-320.52, -635.214));
  line1.AddPoint(Point2d(-268.065, -545.661));
  line1.AddPoint(Point2d(-247.835, -509.644));  // ! leads to selfintersection

  Line line2;
  line2.AddPoint(Point2d(-247.904, -509.757));  // ! leads to selfintersection
  line2.AddPoint(Point2d(-247.284, -508.826));
  line2.AddPoint(Point2d(-246.861, -508.402));
  line2.AddPoint(Point2d(-245.99, -507.853));
  line2.AddPoint(Point2d(-245.376, -507.601));
  line2.AddPoint(Point2d(-244.511, -507.491));
  line2.AddPoint(Point2d(-243.411, -507.495));
  line2.AddPoint(Point2d(-242.827, -507.627));
  line2.AddPoint(Point2d(-241.861, -508.067));  // ! leads to selfintersection

  bool b = bg::intersects(line1.obj_, line2.obj_);
  EXPECT_TRUE(b);

  Line lout12 = ConcatenateLinestring(line1, line2);
  EXPECT_FALSE(bg::intersects(lout12.obj_)) << lout12.ToArray();

  double prol = 1e-3;  // precision for points on line
  double prnl = 1e-2;  // precision for removed points;

  // make sure relevant points are still in there
  // from line1
  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-320.52, -635.214)) < prol);
  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-268.065, -545.661)) < prol);

  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-247.835, -509.644)) < prnl);

  // from line2
  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-247.904, -509.757)) < prnl);

  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-247.284, -508.826)) < prol);
  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-246.861, -508.402)) < prol);
  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-244.511, -507.491)) < prol);
  EXPECT_TRUE(barkg::Distance(lout12, Point2d(-241.861, -508.067)) < prol);

  Line line3;
  line3.AddPoint(Point2d(-208.527 - 525.942));
  line3.AddPoint(Point2d(-241.938 - 508.029));  // ! leads to selfintersection

  Line lout123 = ConcatenateLinestring(lout12, line3);
  EXPECT_FALSE(bg::intersects(lout123.obj_)) << lout123.ToArray();

  // make sure relevant points are still in there
  // from line1
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-320.52, -635.214)) < prol);
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-268.065, -545.661)) < prol);

  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-247.835, -509.644)) < prnl);

  // from line2
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-247.904, -509.757)) < prnl);

  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-247.284, -508.826)) < prol);
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-246.861, -508.402)) < prol);
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-244.511, -507.491)) < prol);
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-241.861, -508.067)) < prol);

  // from line3
  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-208.527 - 525.942)) < prol);

  EXPECT_TRUE(barkg::Distance(lout123, Point2d(-241.938 - 508.029)) < prnl);
}

TEST(buffer, inflate) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;
  using bark::geometry::Pose;

  Polygon in = Polygon(
      Pose(1.25, 0, 0),
      std::vector<Point2d>{Point2d(-1, -1), Point2d(-1, 1), Point2d(3, 1),
                           Point2d(3, -1), Point2d(-1, -1)});

  Polygon out;
  bool suc = BufferPolygon(in, 3, &out);

  Polygon exp = Polygon(
      Pose(1.25, 0, 0),
      std::vector<Point2d>{Point2d(-4, -4), Point2d(-4, 4), Point2d(6, 4),
                           Point2d(6, -4), Point2d(-4, -4)});

  EXPECT_TRUE(Equals(out, exp)) << out.ShapeToString();
  EXPECT_TRUE(suc);

  Polygon s;
  boost::geometry::simplify(out.obj_, s.obj_, 0.1);

  EXPECT_TRUE(Equals(out, s)) << s.ToArray();
}

TEST(generate_car_limousine, vehicleshapes) {
  using namespace bark::geometry;
  using bark::geometry::standard_shapes::GenerateCarLimousine;
  using bark::geometry::standard_shapes::CarLimousine;

  double wb = 3.0568;
  double r = 0.9560;
  Polygon in1 = GenerateCarLimousine(wb, r);
  Polygon in2 = CarLimousine();
  
  // due to numerical issues, this will fail:
  // EXPECT_TRUE(Equals(in1, in2)) << in1.ToArray() << in2.ToArray();
  
  EXPECT_TRUE(std::abs(in1.CalculateArea() - in2.CalculateArea()) < 1e-3);
  EXPECT_TRUE(Distance(in1, in2) < 1e-6);
}

TEST(boundingpolygon, vehicleshapes) {
  using bark::geometry::Point2d;
  using bark::geometry::Polygon;
  using bark::geometry::Pose;
  using bark::geometry::standard_shapes::GenerateCarLimousine;
  using bark::geometry::standard_shapes::GenerateCarRectangle;

  double wb = 2.8;
  double r = 1;
  Polygon in1 = GenerateCarLimousine(wb, r);
  Polygon out1 = CalculateBoundingBoxPolygon(in1);
  EXPECT_TRUE(Within(in1, out1));

  Polygon in2 = GenerateCarRectangle(wb, r);
  Polygon out2 = CalculateBoundingBoxPolygon(in2);
  EXPECT_TRUE(Within(in2, out2));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
