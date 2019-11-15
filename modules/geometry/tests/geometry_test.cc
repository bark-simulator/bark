// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick
// Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "gtest/gtest.h"
#include "modules/geometry/commons.hpp"
#include "modules/geometry/line.hpp"
#include "modules/geometry/polygon.hpp"
#include "modules/geometry/standard_shapes.hpp"

TEST(polygon, base_functionality) {
  using namespace std;
  using namespace modules::geometry;

  // template version; point <--> polygon
  Point2d_t<float> point_1(0.0, 0.0);
  Point2d_t<float> point_2(4.0, 0.0);
  Point2d_t<float> point_3(4.0, 4.0);
  Point2d_t<float> point_4(0.0, 4.0);

  Polygon_t<Point2d_t<float>> polygon;
  polygon.add_point(point_1);
  polygon.add_point(point_2);
  polygon.add_point(point_3);
  polygon.add_point(point_4);

  Point2d_t<float> check_point(5.0, 2.0);

  EXPECT_NEAR(distance(polygon, check_point), 1.0, 0.01);

  // polygon <--> polygon
  Point2d pointf_1(5.0, 2.0);
  Point2d pointf_2(7.5, 0.0);
  Point2d pointf_3(9.0, 2.0);
  Point2d pointf_4(7.5, 4.0);

  Polygon polygon_2;
  polygon_2.add_point(pointf_1);
  polygon_2.add_point(pointf_2);
  polygon_2.add_point(pointf_3);
  polygon_2.add_point(pointf_4);

  EXPECT_NEAR(distance(polygon, polygon_2), 1.0, 0.01);
}

TEST(line, base_functionality) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);

  Point2d_t<float> check_point(2.5, 2.0);

  EXPECT_NEAR(distance(line, check_point), 2.5, 0.01);
}

TEST(geometry, line) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l;  // vertical
  l.add_point(Point2d(0.0f, 0.0f));
  l.add_point(Point2d(0.0f, 10.0f));

  Point2d p = get_point_at_s(l, 5.0f);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 5.0, 0.1);

  p = get_normal_at_s(l, 5.0f);
  EXPECT_NEAR(bg::get<0>(p), -1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = get_nearest_point(l, Point2d(5.0f, 5.0f));
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 5.0, 0.1);

  Line l2;  // horizontal
  l2.add_point(Point2d(0.0f, 0.0f));
  l2.add_point(Point2d(10.0f, 0.0f));

  p = get_point_at_s(l2, 5.0f);
  EXPECT_NEAR(bg::get<0>(p), 5.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = get_normal_at_s(l2, 5.0f);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 1.0, 0.1);

  p = get_nearest_point(l2, Point2d(5.0f, 5.0f));
  EXPECT_NEAR(bg::get<0>(p), 5.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  Line l3;  // 45 deg
  l3.add_point(Point2d(0.0f, 0.0f));
  l3.add_point(Point2d(10.0f, 10.0f));

  p = get_point_at_s(l3, 0.5 * sqrt(200));
  EXPECT_NEAR(bg::get<0>(p), 5.0f, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 5.0f, 0.1);

  p = get_normal_at_s(l3, 0.5 * sqrt(200));
  EXPECT_NEAR(bg::get<0>(p), -0.7, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.7, 0.1);

  Line l4;  // 45 deg
  l4.add_point(Point2d(0.0f, 0.0f));
  l4.add_point(Point2d(-10.0f, 10.0f));

  p = get_normal_at_s(l4, 0.5 * sqrt(200));
  EXPECT_NEAR(bg::get<0>(p), -0.7, 0.1);
  EXPECT_NEAR(bg::get<1>(p), -0.7, 0.1);

  EXPECT_NEAR(get_nearest_s(l, Point2d(5, 5)), 5.0f, 0.1f);
  EXPECT_NEAR(get_nearest_s(l3, Point2d(5, 5)), 0.5 * sqrt(200), 0.1f);
}

TEST(geometry, polygon) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p;
  p.add_point(Point2d(0, 0));
  p.add_point(Point2d(0, 2));
  p.add_point(Point2d(2, 4));
  p.add_point(Point2d(4, 0));
  p.add_point(Point2d(0, 0));

  EXPECT_TRUE(p.Valid());

  std::shared_ptr<Polygon> p2 =
      std::dynamic_pointer_cast<Polygon>(p.rotate(3.14 / 2));
}

TEST(geometry, standard_shapes) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::geometry::standard_shapes;
  namespace bg = boost::geometry;

  Polygon p = CarLimousine();
  ASSERT_TRUE(p.Valid());
}

// poly point collide false
TEST(collision, poly2point1) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p;
  p.add_point(Point2d(0, 0));
  p.add_point(Point2d(0, 2));
  p.add_point(Point2d(2, 4));
  p.add_point(Point2d(4, 0));
  p.add_point(Point2d(0, 0));

  Point2d pt(-2, -2);

  EXPECT_FALSE(Collide(p, pt));
}

// poly point collide true
TEST(collision, poly2point2) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p;
  p.add_point(Point2d(0, 0));
  p.add_point(Point2d(0, 2));
  p.add_point(Point2d(2, 4));
  p.add_point(Point2d(4, 0));
  p.add_point(Point2d(0, 0));

  Point2d pt(1, 1);

  EXPECT_TRUE(Collide(p, pt));
}

// poly poly collide false
TEST(collision, poly2poly1) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p1;
  p1.add_point(Point2d(0, 0));
  p1.add_point(Point2d(0, 1));
  p1.add_point(Point2d(1, 1));
  p1.add_point(Point2d(1, 0));
  p1.add_point(Point2d(0, 0));

  Polygon p2;
  p2.add_point(Point2d(10, 10));
  p2.add_point(Point2d(10, 11));
  p2.add_point(Point2d(11, 11));
  p2.add_point(Point2d(11, 10));
  p2.add_point(Point2d(10, 10));

  EXPECT_FALSE(Collide(p1, p2));

  EXPECT_FALSE(Collide(p2, p1));
}

// poly poly collide overlap true
TEST(collision, poly2poly2) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p1;
  p1.add_point(Point2d(0, 0));
  p1.add_point(Point2d(0, 1));
  p1.add_point(Point2d(1, 1));
  p1.add_point(Point2d(1, 0));
  p1.add_point(Point2d(0, 0));

  Polygon p2;
  p2.add_point(Point2d(0.5, 0.5));
  p2.add_point(Point2d(0.5, 1.5));
  p2.add_point(Point2d(1.5, 1.5));
  p2.add_point(Point2d(1.5, 0.5));
  p2.add_point(Point2d(0.5, 0.5));

  EXPECT_TRUE(Collide(p1, p2));

  EXPECT_TRUE(Collide(p2, p1));
}

// poly poly collide point overlap true
TEST(collision, poly2poly3) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p1;
  p1.add_point(Point2d(0, 0));
  p1.add_point(Point2d(0, 1));
  p1.add_point(Point2d(1, 1));
  p1.add_point(Point2d(1, 0));
  p1.add_point(Point2d(0, 0));

  Polygon p2;
  p2.add_point(Point2d(1, 1));
  p2.add_point(Point2d(1, 2));
  p2.add_point(Point2d(2, 2));
  p2.add_point(Point2d(2, 1));
  p2.add_point(Point2d(1, 1));

  EXPECT_TRUE(Collide(p1, p2));

  EXPECT_TRUE(Collide(p2, p1));
}

// poly line collision no collision
TEST(collision, poly2line1) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p1;
  p1.add_point(Point2d(0, 0));
  p1.add_point(Point2d(0, 1));
  p1.add_point(Point2d(1, 1));
  p1.add_point(Point2d(1, 0));
  p1.add_point(Point2d(0, 0));

  Line l1;
  l1.add_point(Point2d(1.5, 1.5));
  l1.add_point(Point2d(2, 2));

  EXPECT_FALSE(Collide(p1, l1));
}

// poly line collision line intersect
TEST(collision, poly2line2) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p1;
  p1.add_point(Point2d(0, 0));
  p1.add_point(Point2d(0, 1));
  p1.add_point(Point2d(1, 1));
  p1.add_point(Point2d(1, 0));
  p1.add_point(Point2d(0, 0));

  Line l1;
  l1.add_point(Point2d(0.5, 0.5));
  l1.add_point(Point2d(2, 2));

  EXPECT_TRUE(Collide(p1, l1));

  Line l2;
  l2.add_point(Point2d(-0.5, -0.5));
  l2.add_point(Point2d(2, 2));

  EXPECT_TRUE(Collide(p1, l2));
}

// poly line collision point intersect
TEST(collision, poly2line3) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Polygon p1;
  p1.add_point(Point2d(0, 0));
  p1.add_point(Point2d(0, 1));
  p1.add_point(Point2d(1, 1));
  p1.add_point(Point2d(1, 0));
  p1.add_point(Point2d(0, 0));

  Line l1;
  l1.add_point(Point2d(1, 1));
  l1.add_point(Point2d(2, 2));

  EXPECT_TRUE(Collide(p1, l1));
}

// line point collision no intersect
TEST(collision, line2point1) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l1;
  l1.add_point(Point2d(1, 1));
  l1.add_point(Point2d(2, 2));

  Point2d p(0, 0);

  EXPECT_FALSE(Collide(l1, p));
}

// line point collision point intersect
TEST(collision, line2point2) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l1;
  l1.add_point(Point2d(0, 0));
  l1.add_point(Point2d(2, 2));

  Point2d p(0.5, 0.5);

  EXPECT_TRUE(Collide(l1, p));
}

// line point collision end point intersect
TEST(collision, line2point3) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l1;
  l1.add_point(Point2d(1, 1));
  l1.add_point(Point2d(2, 2));

  Point2d p(1, 1);

  EXPECT_TRUE(Collide(l1, p));
}

// line line collision no intersect
TEST(collision, line2line1) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l1;
  l1.add_point(Point2d(1, 1));
  l1.add_point(Point2d(2, 2));

  Line l2;
  l2.add_point(Point2d(3, 3));
  l2.add_point(Point2d(4, 4));

  EXPECT_FALSE(Collide(l1, l2));
}

// line line collision point intersect
TEST(collision, line2line2) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l1;
  l1.add_point(Point2d(1, 1));
  l1.add_point(Point2d(2, 2));

  Line l2;
  l2.add_point(Point2d(1, 1));
  l2.add_point(Point2d(4, 4));

  EXPECT_TRUE(Collide(l1, l2));
  EXPECT_TRUE(Collide(l2, l1));
}

// line line collision point intersect point
TEST(collision, line2line3) {
  using namespace std;
  using namespace modules::geometry;
  namespace bg = boost::geometry;

  Line l1;
  l1.add_point(Point2d(1, 1));
  l1.add_point(Point2d(2, 2));

  Line l2;
  l2.add_point(Point2d(2, 2));
  l2.add_point(Point2d(4, 4));

  EXPECT_TRUE(Collide(l1, l2));
  EXPECT_TRUE(Collide(l2, l1));
}

// car shape collision false
TEST(collision, carshape1) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::geometry::standard_shapes;
  namespace bg = boost::geometry;

  Polygon outline = CarLimousine();
  std::shared_ptr<Polygon> car1 =
      std::dynamic_pointer_cast<Polygon>(outline.transform(Pose(0, 0, 0)));
  std::shared_ptr<Polygon> car2 =
      std::dynamic_pointer_cast<Polygon>(outline.transform(Pose(10, 10, 0)));

  EXPECT_FALSE(Collide(*car1, *car2));
}

// car shape collision true
TEST(collision, carshape2) {
  using namespace std;
  using namespace modules::geometry;
  using namespace modules::geometry::standard_shapes;
  namespace bg = boost::geometry;

  Polygon outline = CarLimousine();
  std::shared_ptr<Polygon> car1 =
      std::dynamic_pointer_cast<Polygon>(outline.transform(Pose(0, 0, 0)));
  std::shared_ptr<Polygon> car2 =
      std::dynamic_pointer_cast<Polygon>(outline.transform(Pose(1, 0, 3.14)));

  // TODO(@hart): HACK
  // EXPECT_TRUE(Collide(car1.get(), car2.get()));
}

TEST(line, s1) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);

  EXPECT_DOUBLE_EQ(line.s_[0], 0.0);
  EXPECT_DOUBLE_EQ(line.s_[1], 1.0);
  EXPECT_DOUBLE_EQ(line.s_[2], 2.0);
}

TEST(line, s2) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(-1.0, 0.0);
  Point2d point_2(0.0, 0.0);
  Point2d point_3(1.0, 0.0);
  Point2d point_4(3.0, 0.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);
  line.add_point(point_4);

  EXPECT_TRUE(line.s_.size() == 4);
  EXPECT_DOUBLE_EQ(line.s_[0], 0.0);
  EXPECT_DOUBLE_EQ(line.s_[1], 1.0);
  EXPECT_DOUBLE_EQ(line.s_[2], 2.0);
  EXPECT_DOUBLE_EQ(line.s_[3], 4.0);
}

TEST(line, get_s_at_pt_1) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);

  Point2d p1 = get_point_at_s(line, 0.0);
  Point2d p2 = get_point_at_s(line, 1.0);
  Point2d p3 = get_point_at_s(line, 2.0);

  Point2d p4 = get_point_at_s(line, 0.5);
  Point2d p5 = get_point_at_s(line, 1.5);

  EXPECT_TRUE(point_1 == p1);
  EXPECT_TRUE(point_2 == p2);
  EXPECT_TRUE(point_3 == p3);

  EXPECT_TRUE(Point2d(0.0, 1.5) == p4);
  EXPECT_TRUE(Point2d(0.0, 2.5) == p5);
}

TEST(line, get_line_from_s_interval) {
  using namespace std;
  using namespace modules::geometry;

  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);

  Line_t<Point2d> line_segment = get_line_from_s_interval(line, 0.5, 1.5);

  Point2d p1 = get_point_at_s(line_segment, 0.0);
  Point2d p2 = get_point_at_s(line_segment, 0.5);
  Point2d p3 = get_point_at_s(line_segment, 1.0);

  EXPECT_TRUE(Point2d(0.0, 1.5) == p1);
  EXPECT_TRUE(point_2 == p2);
  EXPECT_TRUE(Point2d(0.0, 2.5) == p3);
}
TEST(line, get_line_from_s_interval_entire_line) {
  using namespace modules::geometry;

  Point2d point_1(0.0, 0.0);
  Point2d point_2(0.0, 2.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);

  Line_t<Point2d> line_segment = get_line_from_s_interval(line, 0.0, 2.0);

  EXPECT_TRUE(Point2d(0.0, 0.0) == point_1);
  EXPECT_TRUE(Point2d(0.0, 2.0) == point_2);
}

TEST(line, get_nearest_point_1) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 1.0);
  Point2d point_2(0.0, 2.0);
  Point2d point_3(0.0, 3.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);

  Point2d p1 = get_nearest_point(line, point_1);
  Point2d p2 = get_nearest_point(line, point_2);
  Point2d p3 = get_nearest_point(line, point_3);

  Point2d p4 = get_nearest_point(line, Point2d(0, 0));
  Point2d p5 = get_nearest_point(line, Point2d(0, 4));

  Point2d p6 = get_nearest_point(line, Point2d(1, 1));
  Point2d p7 = get_nearest_point(line, Point2d(1, 2.2));

  EXPECT_TRUE(point_1 == p1);
  EXPECT_TRUE(point_2 == p2);
  EXPECT_TRUE(point_3 == p3);

  EXPECT_TRUE(Point2d(0.0, 1.0) == p4);
  EXPECT_TRUE(Point2d(0.0, 3.0) == p5);

  EXPECT_TRUE(Point2d(0.0, 1.0) == p6);
  EXPECT_TRUE(Point2d(0.0, 2.2) == p7);
}

TEST(line, segment_intersection_check_1) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(0.0, 1.0);
  Point2d point_3(0.0, 2.0);
  Point2d point_4(0.0, 3.0);
  Point2d point_5(0.0, 4.0);
  Point2d point_6(0.0, 5.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);
  line.add_point(point_4);
  line.add_point(point_5);
  line.add_point(point_6);

  EXPECT_NEAR(get_segment_end_idx(line, 0.0f), 1, 0.1f);
  EXPECT_NEAR(get_segment_end_idx(line, 3.0f), 4, 0.1f);
  EXPECT_NEAR(get_segment_end_idx(line, 6.0), 5, 0.1f);
}
TEST(line, segment_intersection_tangent_1) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(1.0, 1.0);        // s = sqrt(2)
  Point2d point_3(2.0, 0.0);        // 2 * sqrt(2)
  Point2d point_4(2.0, sqrt(2.0));  // 3* sqrt(2)
  Point2d point_5(2.0 + sqrt(2.0), sqrt(2.0));

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);
  line.add_point(point_4);
  line.add_point(point_5);

  EXPECT_NEAR(get_tangent_angle_at_s(line, sqrt(2)), 0, 0.1f);
  EXPECT_NEAR(get_tangent_angle_at_s(line, 2 * sqrt(2)), (1.0 / 8.0) * 3.141,
              0.1f);

  // template
  Point2d point_6(0.0, 0.0);
  Point2d point_7(0, 1.0);
  Point2d point_8(0.0, 2.0);
  Point2d point_9(0, 3.0);
  Point2d point_10(0.0, 4.0);

  Line_t<Point2d> line2;

  line2.add_point(point_6);
  line2.add_point(point_7);
  line2.add_point(point_8);
  line2.add_point(point_9);
  line2.add_point(point_10);

  EXPECT_NEAR(get_tangent_angle_at_s(line2, 0.25), (1.0 / 2.0) * 3.141, 0.1f);
  EXPECT_NEAR(get_tangent_angle_at_s(line2, 0.5), (1.0 / 2.0) * 3.141, 0.1f);
}

TEST(line, segment_get_normal_1) {
  using namespace std;
  using namespace modules::geometry;

  // template
  Point2d point_1(0.0, 0.0);
  Point2d point_2(1.0, 0.0);
  Point2d point_3(1.0, 1.0);
  Point2d point_4(0.0, 1.0);

  Line_t<Point2d> line;

  line.add_point(point_1);
  line.add_point(point_2);
  line.add_point(point_3);
  line.add_point(point_4);
  line.add_point(point_1);

  Point2d p = get_normal_at_s(line, 0.5f);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 1.0, 0.1);

  p = get_normal_at_s(line, 1.5f);
  EXPECT_NEAR(bg::get<0>(p), -1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = get_normal_at_s(line, 2.5f);
  EXPECT_NEAR(bg::get<0>(p), 0.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), -1.0, 0.1);

  p = get_normal_at_s(line, 3.5f);
  EXPECT_NEAR(bg::get<0>(p), 1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);

  p = get_normal_at_s(line, 4.0f);
  EXPECT_NEAR(bg::get<0>(p), 1.0, 0.1);
  EXPECT_NEAR(bg::get<1>(p), 0.0, 0.1);
}

TEST(optimizer, shrink_polygon) {
  using modules::geometry::Point2d;
  using modules::geometry::Polygon;

  Polygon polygon;
  polygon.add_point(Point2d(0, 0));
  polygon.add_point(Point2d(4, 0));
  polygon.add_point(Point2d(4, 2));
  polygon.add_point(Point2d(6, 2));
  polygon.add_point(Point2d(6, 0));
  polygon.add_point(Point2d(10, 0));
  polygon.add_point(Point2d(10, 4));
  polygon.add_point(Point2d(0, 4));
  polygon.add_point(Point2d(0, 0));

  Polygon shrunk_polygon;
  ShrinkPolygon(polygon, -1, shrunk_polygon);

  Polygon expected_shrunk_polygon;
  polygon.add_point(Point2d(1, 1));
  polygon.add_point(Point2d(3, 1));
  polygon.add_point(Point2d(3, 3));
  polygon.add_point(Point2d(7, 3));
  polygon.add_point(Point2d(7, 1));
  polygon.add_point(Point2d(7, 1));
  polygon.add_point(Point2d(9, 3));
  polygon.add_point(Point2d(9, 3));
  polygon.add_point(Point2d(1, 1));

  ASSERT_TRUE(equals(expected_shrunk_polygon, shrunk_polygon));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
