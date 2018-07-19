#include <gtest/gtest.h>

#include <cmath>

#include "Point.hpp"

TEST(PointTest, defaultIsOrigin)
{
   Point p;
   EXPECT_EQ(Point(0,0,0), p);
}

TEST(PointTest, xyContructor)
{
   Point p(1,2);
   EXPECT_EQ(1, p.GetX());
   EXPECT_EQ(2, p.GetY());
   EXPECT_EQ(0, p.GetZ());
}

TEST(PointTest, xyzConstructor)
{
   Point p(1,2,3);
   EXPECT_EQ(1, p.GetX());
   EXPECT_EQ(2, p.GetY());
   EXPECT_EQ(3, p.GetZ());
}

TEST(PointTest, setters)
{
   Point p(1,2,3);
   p.SetX(-10);
   p.SetY(-20);
   p.SetZ(-30);
   EXPECT_EQ(-10, p.GetX());
   EXPECT_EQ(-20, p.GetY());
   EXPECT_EQ(-30, p.GetZ());
}

TEST(PointTest, distanceToSelfIsZero)
{
   Point p1(1,2,-3);
   Point p2;
   Point p3(1,0,0);
   Point p4(0,1,0);
   Point p5(0,0,1);

   EXPECT_EQ(0, p1.Distance(p1));
   EXPECT_EQ(0, p2.Distance(p2));
   EXPECT_EQ(0, p3.Distance(p3));
   EXPECT_EQ(0, p4.Distance(p4));
   EXPECT_EQ(0, p5.Distance(p5));
}

TEST(PointTest, distanceIsCommutative)
{
   Point p1(1,2,3);
   Point p2(41, 0, 1);
   EXPECT_EQ(p1.Distance(p2), p2.Distance(p1));
}

TEST(PointTest, distanceBetweenUnit)
{
   Point p0(0,0,0);
   Point p1(1,0,0);
   Point p2(0,1,0);
   Point p3(0,0,1);
   Point p4(-1,0,0);

   EXPECT_EQ(1, p1.Distance(p0));
   EXPECT_EQ(1, p0.Distance(p2));
   EXPECT_EQ(1, p0.Distance(p3));
   EXPECT_EQ(1, p0.Distance(p4));

   EXPECT_EQ(sqrt(2), p1.Distance(p2));
   EXPECT_EQ(sqrt(2), p1.Distance(p3));
   EXPECT_EQ(sqrt(2), p2.Distance(p3));

   EXPECT_EQ(2, p4.Distance(p1));
}
