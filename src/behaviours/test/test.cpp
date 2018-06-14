#include "BehaviorManager.hpp"
#include "RobotInterface.hpp"
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
