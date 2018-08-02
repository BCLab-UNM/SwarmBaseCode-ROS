#include "BehaviorManager.hpp"
#include "SwarmieInterface.hpp"
#include "ObstacleBehavior.hpp"

#include "gmock/gmock.h"
#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>
#include <algorithm> // count_if

/**
 * Simple hash function to deterministically create many different tag
 * IDs.
 */
int simple_hash(int x)
{
   return (1333333 * x) >> (32 - 16);
}

TEST(BehaviorManager, noBehaviors) {
   // The default behavior is to sit and do nothing after lowering and
   // opening the gripper.
   BehaviorManager manager;
   SwarmieAction a = manager.NextAction(SwarmieSensors());
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_EQ(0, a.GetVelocity().GetAngularMagnitude());
   EXPECT_EQ(WristControl::DOWN, a.WristCommand());
   EXPECT_EQ(GripperControl::OPEN, a.GripperCommand());
}

TEST(BehaviorManager, constantBehavior) {
   BehaviorManager manager;
   SwarmieAction action;
   action.SetAction(core::VelocityAction(AngularVelocity(0, 0, 1.0)));
   action.SetWrist(WristControl::UP);
   action.SetGrip(GripperControl::CLOSED);
   Constant c(action);
   manager.RegisterBehavior(&c);
   SwarmieAction a = manager.NextAction(SwarmieSensors());
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_EQ(1.0, action.GetVelocity().GetYaw());
   EXPECT_EQ(a.WristCommand(), action.WristCommand());
   EXPECT_EQ(a.GripperCommand(), action.GripperCommand());
}

TEST(Constant, constantBehavior) {
   SwarmieAction action;
   action.SetAction(core::VelocityAction(AngularVelocity(0, 0, -1)));
   action.SetWrist(WristControl::UP);
   action.SetGrip(GripperControl::CLOSED);
   Constant c(action);
   SwarmieAction a = c.GetAction();
   EXPECT_EQ(a.GetVelocity().GetAngularMagnitude(), action.GetVelocity().GetAngularMagnitude());
   EXPECT_EQ(a.GetVelocity().GetLinearMagnitude(), action.GetVelocity().GetLinearMagnitude());
   EXPECT_EQ(a.GetVelocity().GetYaw(), action.GetVelocity().GetYaw());
   EXPECT_EQ(a.WristCommand(), action.WristCommand());
   EXPECT_EQ(a.GripperCommand(), action.GripperCommand());
}

TEST(Constant, constantBehaviorNoChangeAfterUpdate) {
   SwarmieAction action;
   action.SetAction(core::VelocityAction(AngularVelocity(0, 0, -1)));
   action.SetWrist(WristControl::UP);
   action.SetGrip(GripperControl::CLOSED);
   Constant c(action);
   c.Update(SwarmieSensors(), SwarmieAction());
   SwarmieAction a = c.GetAction();
   EXPECT_EQ(a.GetVelocity().GetAngularMagnitude(), action.GetVelocity().GetAngularMagnitude());
   EXPECT_EQ(a.GetVelocity().GetLinearMagnitude(), action.GetVelocity().GetLinearMagnitude());
   EXPECT_EQ(a.GetVelocity().GetYaw(), action.GetVelocity().GetYaw());
   EXPECT_EQ(a.WristCommand(), action.WristCommand());
   EXPECT_EQ(a.GripperCommand(), action.GripperCommand());
}

TEST(Constant, constantBehaviorNoChangeAfterLLActionUpdate) {
   SwarmieAction action;
   action.SetAction(core::VelocityAction(AngularVelocity(0, 0, -1)));
   action.SetWrist(WristControl::UP);
   action.SetGrip(GripperControl::CLOSED);
   Constant c(action);
   SwarmieAction action2;
   action2.SetAction(core::VelocityAction(AngularVelocity(0, 0, 0.75)));
   action2.SetWrist(WristControl::DOWN_2_3);
   action2.SetGrip(GripperControl::OPEN);
   c.Update(SwarmieSensors(), action2);
   SwarmieAction a = c.GetAction();
   EXPECT_EQ(a.GetVelocity().GetAngularMagnitude(), action.GetVelocity().GetAngularMagnitude());
   EXPECT_EQ(a.GetVelocity().GetLinearMagnitude(), action.GetVelocity().GetLinearMagnitude());
   EXPECT_EQ(a.GetVelocity().GetYaw(), action.GetVelocity().GetYaw());
   EXPECT_EQ(a.WristCommand(), action.WristCommand());
   EXPECT_EQ(a.GripperCommand(), action.GripperCommand());
}

TEST(SwarmieSensors, setSonar) {
   SwarmieSensors sensors;
   sensors.SetLeftSonar(0.1);
   sensors.SetRightSonar(0.2);
   sensors.SetCenterSonar(2.4);
   EXPECT_EQ(0.1, sensors.GetLeftSonar());
   EXPECT_EQ(0.2, sensors.GetRightSonar());
   EXPECT_EQ(2.4, sensors.GetCenterSonar());
}

TEST(SwarmieSensors, initialTagDetections) {
   // initially should not detect any tags.
   SwarmieSensors sensors;
   EXPECT_EQ(0, sensors.GetTags().size());
}

TEST(SwarmieSensors, setTagsOne) {
   SwarmieSensors sensors;
   boost::math::quaternion<double> q(1.2, 1.2, 1.2, 5.2);
   Tag t(1, 2, 3, 4, q);
   sensors.DetectedTag(t);
   std::vector<Tag> tags = sensors.GetTags();
   EXPECT_EQ(1, sensors.GetTags().size());
   Tag t_out = sensors.GetTags()[0];
   EXPECT_EQ(q, sensors.GetTags()[0].GetOrientation());
   EXPECT_EQ(t.GetX(), t_out.GetX());
   EXPECT_EQ(t.GetY(), t_out.GetY());
   EXPECT_EQ(t.GetZ(), t_out.GetZ());
}

TEST(SwarmieSensors, setTagsMany)
{
   const int MAX_TAG_ID = 257;
   SwarmieSensors sensors;
   boost::math::quaternion<double> q(0.2, 10.4, -1.2, 0.3);
   std::vector<Tag> tags_in;
   for(int i = 0; i <= 1024; i++)
   {
      Tag t(simple_hash(i) % MAX_TAG_ID, i, i, i, q);
      sensors.DetectedTag(t);
      tags_in.push_back(t);
   }
   EXPECT_EQ(tags_in.size(), sensors.GetTags().size());
   std::vector<Tag> tags_out = sensors.GetTags();
   for(int i = 0; i < MAX_TAG_ID; i++)
   {
      int i_out = std::count_if(tags_out.begin(), tags_out.end(), [i](Tag t) { return i == t.GetId(); } );
      int i_in  = std::count_if(tags_in.begin(), tags_in.end(), [i](Tag t) { return i == t.GetId(); });
      ASSERT_EQ(i_in, i_out);
   }
}

TEST(SwarmieSensors, deadReckoningPosition)
{
   SwarmieSensors sensors;
   sensors.SetDeadReckoningPosition(Point(-1.23, 2.659));
   sensors.SetGPSFusedPosition(Point(18.12, -100.2));
   EXPECT_EQ(sensors.GetDeadReckoningPosition(), Point(-1.23,2.659));
   EXPECT_EQ(sensors.GetGPSFusedPosition(), Point(18.12, -100.2));
}

TEST(SwarmieSensors, heading)
{
   SwarmieSensors sensors;
   sensors.SetHeading(Heading(0));
   EXPECT_EQ(sensors.GetHeading(), Heading(0));
   sensors.SetHeading(Heading(2*M_PI/3));
   EXPECT_EQ(sensors.GetHeading(), Heading(2*M_PI/3));
}

int main(int argc, char** argv)
{
   testing::InitGoogleTest(&argc, argv);
//   ::testing::InitGoogleMock(&argc, argv);
   return RUN_ALL_TESTS();
}
