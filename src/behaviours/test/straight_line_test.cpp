#include <gtest/gtest.h>

#include "test_util.h"

#include "StraightLineBehavior.hpp"

class StraightLineBehaviorTest : public testing::Test
{
protected:
   StraightLineBehavior sl;
   StraightLineBehaviorTest() : sl() {
      Action action;
      action.drive.left = 0;
      action.drive.right = 0;
      sl.SetLowerLevelAction(action);
      sl.Update();
   }
};

TEST_F(StraightLineBehaviorTest, movesForward)
{
   EXPECT_TRUE(is_moving(sl.GetAction()));
   EXPECT_FALSE(is_turning_left(sl.GetAction()));
   EXPECT_FALSE(is_turning_right(sl.GetAction()));
}

TEST_F(StraightLineBehaviorTest, isTurningKeepsTurning)
{
   Action turn;
   turn.drive.left = 100;
   turn.drive.right = 20;
   sl.SetLowerLevelAction(turn);
   sl.Update();
   EXPECT_EQ(turn.drive.left, sl.GetAction().drive.left);
   EXPECT_EQ(turn.drive.right, sl.GetAction().drive.right);
   turn.drive.left = 0;
   turn.drive.right = 0;
   sl.SetLowerLevelAction(turn);
   sl.Update();
   EXPECT_TRUE(is_moving(sl.GetAction()));
   EXPECT_EQ(sl.GetAction().drive.left, sl.GetAction().drive.right);
}

TEST_F(StraightLineBehaviorTest, isRotatingKeepsRotating)
{
   Action turn;
   turn.drive.left = -60;
   turn.drive.right = 60;
   sl.SetLowerLevelAction(turn);
   sl.Update();
   EXPECT_EQ(turn.drive.left, sl.GetAction().drive.left);
   EXPECT_EQ(turn.drive.right, sl.GetAction().drive.right);
}
