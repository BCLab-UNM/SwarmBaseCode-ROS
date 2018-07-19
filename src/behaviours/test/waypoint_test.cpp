#include <gtest/gtest.h>
#include "Action.hpp"

using namespace core;

TEST(WaypointActionTest, gettersAndSetters)
{
   Point p1(0,0,0);
   WaypointAction w(p1);
   w.SetX(1);
   w.SetY(2);
   w.SetZ(3);
   w.SetSpeed(10);
   w.SetTolerance(0.6);
   EXPECT_EQ(1,   w.GetX());
   EXPECT_EQ(2,   w.GetY());
   EXPECT_EQ(3,   w.GetZ());
   EXPECT_EQ(10,  w.GetSpeed());
   EXPECT_EQ(0.6, w.GetTolerance());
   EXPECT_EQ(Point(1,2,3), w.GetWaypoint());
}

TEST(WaypointActionTest, isAtSamePoint)
{
   Point p0(0,0,0);
   Point p1(1,-2,3);
   WaypointAction w0(p0);
   WaypointAction w1(p1);

   EXPECT_TRUE(w0.IsAt(p0));
   EXPECT_TRUE(w1.IsAt(p1));
}

TEST(WaypointActionTest, isNotAt)
{
   Point p1(1,2,3);
   Point p2(-1,-2,-3);
   Point p3(-1,2,3);
   WaypointAction w(p1);

   EXPECT_FALSE(w.IsAt(p2));
   EXPECT_FALSE(w.IsAt(p3));
}

TEST(WaypointActionTest, isAtToleranceZero)
{
   Point p0(0,0,0);
   WaypointAction w(p0);
   w.SetTolerance(0.0);
   Point p(0.00001, 0.000001, 0);
   EXPECT_FALSE(w.IsAt(p));
   EXPECT_FALSE(w.IsAt(Point(1,0,0)));
   EXPECT_FALSE(w.IsAt(Point(0,1,0)));
   EXPECT_FALSE(w.IsAt(Point(0,0,1)));
   EXPECT_TRUE(w.IsAt(p0));
}

TEST(WaypointActionTest, isAtNearby)
{
   WaypointAction w(Point(-0.5,-0.5,-0.5));
   w.SetTolerance(1.0);
   EXPECT_TRUE(w.IsAt(Point(0,-0.5,-0.5)));
   EXPECT_TRUE(w.IsAt(Point(0.5, -0.5, -0.5)));
   EXPECT_TRUE(w.IsAt(Point(0,0,0)));
}
