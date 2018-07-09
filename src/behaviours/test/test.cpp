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
   Action a = manager.NextAction();
   EXPECT_EQ(0, a.drive.left);
   EXPECT_EQ(0, a.drive.right);
   EXPECT_EQ(WristControl::DOWN, a.wrist);
   EXPECT_EQ(GripperControl::OPEN, a.grip);
}

TEST(BehaviorManager, constantBehavior) {
   BehaviorManager manager;
   Action action;
   action.drive.left = -100;
   action.drive.right = 100;
   action.wrist = WristControl::UP;
   action.grip  = GripperControl::CLOSED;
   Constant c(action);
   manager.RegisterBehavior(&c);
   Action a = manager.NextAction();
   EXPECT_EQ(a.drive.left, action.drive.left);
   EXPECT_EQ(a.drive.right, action.drive.right);
   EXPECT_EQ(a.wrist, action.wrist);
   EXPECT_EQ(a.grip, action.grip);
}

TEST(Constant, constantBehavior) {
   Action action;
   action.drive.left = -100;
   action.drive.right = 100;
   action.wrist = WristControl::UP;
   action.grip  = GripperControl::CLOSED;
   Constant c(action);
   Action a = c.GetAction();
   EXPECT_EQ(a.drive.left, action.drive.left);
   EXPECT_EQ(a.drive.right, action.drive.right);
   EXPECT_EQ(a.wrist, action.wrist);
   EXPECT_EQ(a.grip, action.grip);
}

TEST(Constant, constantBehaviorNoChangeAfterUpdate) {
   Action action;
   action.drive.left = -100;
   action.drive.right = 100;
   action.wrist = WristControl::UP;
   action.grip  = GripperControl::CLOSED;
   Constant c(action);
   c.Update();
   Action a = c.GetAction();
   EXPECT_EQ(a.drive.left, action.drive.left);
   EXPECT_EQ(a.drive.right, action.drive.right);
   EXPECT_EQ(a.wrist, action.wrist);
   EXPECT_EQ(a.grip, action.grip);
}

TEST(Constant, constantBehaviorNoChangeAfterLLActionUpdate) {
   Action action;
   action.drive.left = -100;
   action.drive.right = 100;
   action.wrist = WristControl::UP;
   action.grip  = GripperControl::CLOSED;
   Constant c(action);
   Action action2;
   action2.drive.left = 0;
   action2.drive.right = 10;
   action2.wrist = WristControl::DOWN_2_3;
   action2.grip = GripperControl::OPEN;
   c.SetLowerLevelAction(action2);
   c.Update();
   Action a = c.GetAction();
   EXPECT_EQ(a.drive.left, action.drive.left);
   EXPECT_EQ(a.drive.right, action.drive.right);
   EXPECT_EQ(a.wrist, action.wrist);
   EXPECT_EQ(a.grip, action.grip);
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

int main(int argc, char** argv)
{
   testing::InitGoogleTest(&argc, argv);
   ::testing::InitGoogleMock(&argc, argv);
   return RUN_ALL_TESTS();
}
