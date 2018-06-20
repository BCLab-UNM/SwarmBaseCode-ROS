#include "BehaviorManager.hpp"
#include "SwarmieInterface.hpp"
#include <gtest/gtest.h>

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

int main(int argc, char** argv)
{
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "tester");
   ros::NodeHandle nh;
   return RUN_ALL_TESTS();
}
