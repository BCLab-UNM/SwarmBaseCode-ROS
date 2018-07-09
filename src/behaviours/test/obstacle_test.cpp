#include <gtest/gtest.h>
#include "test_util.h"

#include "ObstacleBehavior.hpp"
#include "SwarmieSensors.hpp"
#include "MockTimer.hpp"

// Test fixture for obstacle behavior
class ObstacleBehaviorTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   MockTimer timer;
   ObstacleBehavior obs;
   
   ObstacleBehaviorTest() : obs(&sensors, &timer) {
      sensors.SetLeftSonar(3.2);
      sensors.SetRightSonar(3.2);
      sensors.SetCenterSonar(3.2);
      obs.Update();
   }
};

TEST_F(ObstacleBehaviorTest, allFar)
{
   obs.Update();
   Action a = obs.GetAction();
   EXPECT_FALSE(is_moving(a));
}

// TODO: Test the of ObstacleBehavior behavior when each US is at a
// mid/low value (0.8, 0.7, 0.5, 0.3) In each case the movement should
// be non-negligible
TEST_F(ObstacleBehaviorTest, leftSonarTriggersMovement)
{
   Action a;

   sensors.SetLeftSonar(0.3);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetLeftSonar(0.5);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetLeftSonar(0.7);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetLeftSonar(0.8);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(ObstacleBehaviorTest, rightSonarTriggersMovement)
{
   Action a;

   sensors.SetRightSonar(0.3);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetRightSonar(0.5);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetRightSonar(0.7);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
 
   sensors.SetRightSonar(0.8);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(ObstacleBehaviorTest, centerSonarTriggersMovement)
{
   Action a;

   sensors.SetCenterSonar(0.3);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetCenterSonar(0.5);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetCenterSonar(0.7);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
 
   sensors.SetCenterSonar(0.79);
   obs.Update();
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
}


// TODO: If any one sonar reads below 0.3 then left + right = 0 &&
// left /= 0 && right /= 0. ie. the action is to turn in place.
TEST_F(ObstacleBehaviorTest, leftTriggersTurnaround)
{
   Action a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetLeftSonar(0.29);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, rightTriggersTurnaround)
{
   Action a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetRightSonar(0.29);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, centerSonarTriggersTurnaround)
{
   Action a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetCenterSonar(0.29);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, leftAndRightTriggerTurnaround)
{
   Action a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetLeftSonar(0.2);
   sensors.SetRightSonar(0.28);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, leftAndCenterTriggerTurnaround)
{
   Action a;

   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetLeftSonar(0.2);
   sensors.SetCenterSonar(0.28);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, centerAndRightTriggerTurnaround)
{
   Action a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetCenterSonar(0.2);
   sensors.SetRightSonar(0.28);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, allTriggerTurnaround)
{
   Action a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetCenterSonar(0.2);
   sensors.SetRightSonar(0.28);
   sensors.SetLeftSonar(0.1);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, turnaroundContinuesIfSonarStillReadsClose)
{
   // Set expectations
   using ::testing::Return;
   EXPECT_CALL(timer, Expired())
      .Times(1)
      .WillOnce(Return(true));
   // the timer should be started twice
   EXPECT_CALL(timer, StartOnce())
      .Times(2);

   // do the test, we should be turning around
   sensors.SetLeftSonar(0.1);
   obs.Update();
   Action a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));

   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST_F(ObstacleBehaviorTest, turnaroundStopsAfterTimerExpires)
{
   using ::testing::Return;
   EXPECT_CALL(timer, StartOnce()).Times(1);
   EXPECT_CALL(timer, Expired())
      .Times(1)
      .WillRepeatedly(Return(true));

   sensors.SetRightSonar(0.1);
   obs.Update();
   Action a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));

   sensors.SetRightSonar(3.2);
   obs.Update();
   EXPECT_FALSE(is_moving(obs.GetAction()));
}