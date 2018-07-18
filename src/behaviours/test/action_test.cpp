#include <gtest/gtest.h>

#include "action_util.hpp"

TEST(ActionTest, defaultVelocityActionDoesNothing)
{
   VelocityAction va;
   ASSERT_TRUE(is_nil(va));
}

TEST(AngularVelocityTest, additiveInverse)
{
   AngularVelocity av1(0,0,1);
   AngularVelocity av2(0,0,-1);

   AngularVelocity av3(0,1,0);
   AngularVelocity av4(0,-1,0);

   AngularVelocity av5(1,0,0);
   AngularVelocity av6(-1,0,0);

   EXPECT_EQ(AngularVelocity(0,0,0), av1 + av2);
   EXPECT_EQ(AngularVelocity(0,0,0), av3 + av4);
   EXPECT_EQ(AngularVelocity(0,0,0), av5 + av6);
}

TEST(AngularVelocityTest, multiplicationByZero)
{
   AngularVelocity av(1,-2,3);
   EXPECT_EQ(AngularVelocity(0,0,0), 0 * av);
}

TEST(AngularVelocityTest, multiplicationByOne)
{
   AngularVelocity av(-2,6,3);
   EXPECT_EQ(av, 1*av);
}

TEST(AngularVelocityTest, doubleVelocity)
{
   AngularVelocity av(3,7,-10);
   EXPECT_EQ(AngularVelocity(6,14,-20), 2*av);
}

TEST(AngularVelocityTest, multiplicationByNegativeOne)
{
   AngularVelocity av(5,-2,3);
   AngularVelocity v = av + (-1 * av);
   EXPECT_EQ(AngularVelocity(0,0,0), v) << "<0,0,0> /= "
                                        << "<"
                                        << v.GetRoll()
                                        << ","
                                        << v.GetPitch()
                                        << ","
                                        << v.GetYaw()
                                        << ">"
                                        << std::endl;
}

TEST(AngularVelocityTest, unitMagnitude)
{
   AngularVelocity av1(1,0,0);
   AngularVelocity av2(-1,0,0);
   AngularVelocity av3(0,1,0);
   AngularVelocity av4(0,-1,0);
   AngularVelocity av5(0,0,1);
   AngularVelocity av6(0,0,-1);
   EXPECT_EQ(1, av1.GetMagnitude());
   EXPECT_EQ(1, av2.GetMagnitude());
   EXPECT_EQ(1, av3.GetMagnitude());
   EXPECT_EQ(1, av4.GetMagnitude());
   EXPECT_EQ(1, av5.GetMagnitude());
   EXPECT_EQ(1, av6.GetMagnitude());
}

TEST(AngularVelocityTest, inequality)
{
   AngularVelocity av1(0,0,1);
   AngularVelocity av2(0,1,0);
   AngularVelocity av3(1,0,0);
   EXPECT_FALSE(av1 == av2);
   EXPECT_FALSE(av2 == av3);
   EXPECT_FALSE(av3 == av1);
}

TEST(AngularVelocityTest, zeroMagnitude)
{
   AngularVelocity av(0,0,0);
   EXPECT_EQ(0, av.GetMagnitude());
}

TEST(AngularVelocityTest, subtraction)
{
   AngularVelocity av(3,2,-6);
   EXPECT_EQ(AngularVelocity(0,0,0), av - av);
}

TEST(AngularVelocityTest, getters)
{
   AngularVelocity v(1,2,3);
   EXPECT_EQ(1, v.GetRoll());
   EXPECT_EQ(2, v.GetPitch());
   EXPECT_EQ(3, v.GetYaw());
}

TEST(AngularVelocityTest, setters)
{
   AngularVelocity v1(0,0,0);
   AngularVelocity v2(0,0,0);
   AngularVelocity v3(0,0,0);

   v1.SetRoll(1);
   EXPECT_EQ(1, v1.GetRoll());
   EXPECT_EQ(0, v1.GetPitch());
   EXPECT_EQ(0, v1.GetYaw());

   v2.SetPitch(1);
   EXPECT_EQ(0, v2.GetRoll());
   EXPECT_EQ(1, v2.GetPitch());
   EXPECT_EQ(0, v2.GetYaw());

   v3.SetYaw(1);
   EXPECT_EQ(0, v3.GetRoll());
   EXPECT_EQ(0, v3.GetPitch());
   EXPECT_EQ(1, v3.GetYaw());
}

/// LinearVelocityTests
TEST(LinearVelocityTest, additiveInverse)
{
   LinearVelocity lv1(0,0,1);
   LinearVelocity lv2(0,0,-1);

   LinearVelocity lv3(0,1,0);
   LinearVelocity lv4(0,-1,0);

   LinearVelocity lv5(1,0,0);
   LinearVelocity lv6(-1,0,0);

   EXPECT_EQ(LinearVelocity(0,0,0), lv1 + lv2);
   EXPECT_EQ(LinearVelocity(0,0,0), lv3 + lv4);
   EXPECT_EQ(LinearVelocity(0,0,0), lv5 + lv6);
}

TEST(LinearVelocityTest, multiplicationByZero)
{
   LinearVelocity lv(1,-2,3);
   EXPECT_EQ(LinearVelocity(0,0,0), 0 * lv);
}

TEST(LinearVelocityTest, multiplicationByOne)
{
   LinearVelocity lv(-2,6,3);
   EXPECT_EQ(lv, 1*lv);
}

TEST(LinearVelocityTest, doubleVelocity)
{
   LinearVelocity lv(3,7,-10);
   EXPECT_EQ(LinearVelocity(6,14,-20), 2*lv);
}

TEST(LinearVelocityTest, multiplicationByNegativeOne)
{
   LinearVelocity lv(5,-2,3);
   LinearVelocity v = lv + (-1 * lv);
   EXPECT_EQ(LinearVelocity(0,0,0), v) << "<0,0,0> /= "
                                       << "<"
                                       << v.GetX()
                                       << ","
                                       << v.GetY()
                                       << ","
                                       << v.GetZ()
                                       << ">"
                                       << std::endl;
}

TEST(LinearVelocityTest, unitMagnitude)
{
   LinearVelocity lv1(1,0,0);
   LinearVelocity lv2(-1,0,0);
   LinearVelocity lv3(0,1,0);
   LinearVelocity lv4(0,-1,0);
   LinearVelocity lv5(0,0,1);
   LinearVelocity lv6(0,0,-1);
   EXPECT_EQ(1, lv1.GetMagnitude());
   EXPECT_EQ(1, lv2.GetMagnitude());
   EXPECT_EQ(1, lv3.GetMagnitude());
   EXPECT_EQ(1, lv4.GetMagnitude());
   EXPECT_EQ(1, lv5.GetMagnitude());
   EXPECT_EQ(1, lv6.GetMagnitude());
}

TEST(LinearVelocityTest, zeroMagnitude)
{
   LinearVelocity lv(0,0,0);
   EXPECT_EQ(0, lv.GetMagnitude());
}

TEST(LinearVelocityTest, inequality)
{
   LinearVelocity lv1(0,0,1);
   LinearVelocity lv2(0,1,0);
   LinearVelocity lv3(1,0,0);
   EXPECT_FALSE(lv1 == lv2);
   EXPECT_FALSE(lv2 == lv3);
   EXPECT_FALSE(lv3 == lv1);
}

TEST(LinearVelocityTest, subtraction)
{
   LinearVelocity lv(3,2,-6);
   EXPECT_EQ(LinearVelocity(0,0,0), lv - lv);
}

TEST(LinearVelocityTest, getters)
{
   LinearVelocity v(1,2,3);
   EXPECT_EQ(1, v.GetX());
   EXPECT_EQ(2, v.GetY());
   EXPECT_EQ(3, v.GetZ());
}

TEST(LinearVelocityTest, setters)
{
   LinearVelocity v1(0,0,0);
   LinearVelocity v2(0,0,0);
   LinearVelocity v3(0,0,0);

   v1.SetX(1);
   EXPECT_EQ(1, v1.GetX());
   EXPECT_EQ(0, v1.GetY());
   EXPECT_EQ(0, v1.GetZ());

   v2.SetY(1);
   EXPECT_EQ(0, v2.GetX());
   EXPECT_EQ(1, v2.GetY());
   EXPECT_EQ(0, v2.GetZ());

   v3.SetZ(1);
   EXPECT_EQ(0, v3.GetX());
   EXPECT_EQ(0, v3.GetY());
   EXPECT_EQ(1, v3.GetZ());
}

TEST(LinearVelocityTest, constructors)
{
   LinearVelocity v1;
   LinearVelocity v2(1);
   LinearVelocity v3(2,3);
   EXPECT_EQ(LinearVelocity(0,0,0), v1);
   EXPECT_EQ(LinearVelocity(1,0,0), v2);
   EXPECT_EQ(LinearVelocity(2,3,0), v3);
}
