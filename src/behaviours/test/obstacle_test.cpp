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
   
   ObstacleBehaviorTest() : obs(&timer) {
      sensors.SetLeftSonar(3.2);
      sensors.SetRightSonar(3.2);
      sensors.SetCenterSonar(3.2);
      obs.Update(sensors, SwarmieAction());
   }
};

TEST_F(ObstacleBehaviorTest, allFar)
{
   obs.Update(sensors, SwarmieAction());
   SwarmieAction a = obs.GetAction();
   EXPECT_FALSE(is_moving(a));
}

// TODO: Test the of ObstacleBehavior behavior when each US is at a
// mid/low value (0.8, 0.7, 0.5, 0.3) In each case the movement should
// be non-negligible
TEST_F(ObstacleBehaviorTest, leftSonarTriggersMovement)
{
   SwarmieAction a;

   sensors.SetLeftSonar(0.3);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetLeftSonar(0.5);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetLeftSonar(0.7);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetLeftSonar(0.8);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(ObstacleBehaviorTest, rightSonarTriggersMovement)
{
   SwarmieAction a;

   sensors.SetRightSonar(0.3);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetRightSonar(0.5);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetRightSonar(0.7);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
 
   sensors.SetRightSonar(0.8);
   obs.Update(sensors,SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(ObstacleBehaviorTest, centerSonarTriggersMovement)
{
   SwarmieAction a;

   sensors.SetCenterSonar(0.3);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a)) << a.GetVelocity().GetAngularMagnitude();

   sensors.SetCenterSonar(0.5);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));

   sensors.SetCenterSonar(0.7);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
 
   sensors.SetCenterSonar(0.79);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(ObstacleBehaviorTest, leftTriggersTurnaround)
{
   SwarmieAction a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetLeftSonar(0.29);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, rightTriggersTurnaround)
{
   SwarmieAction a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetRightSonar(0.29);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, centerSonarTriggersTurnaround)
{
   SwarmieAction a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetCenterSonar(0.29);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, leftAndRightTriggerTurnaround)
{
   SwarmieAction a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetLeftSonar(0.2);
   sensors.SetRightSonar(0.28);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, leftAndCenterTriggerTurnaround)
{
   SwarmieAction a;

   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetLeftSonar(0.2);
   sensors.SetCenterSonar(0.28);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, centerAndRightTriggerTurnaround)
{
   SwarmieAction a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetCenterSonar(0.2);
   sensors.SetRightSonar(0.28);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, allTriggerTurnaround)
{
   SwarmieAction a;
   EXPECT_CALL(timer, StartOnce()).Times(1);

   sensors.SetCenterSonar(0.2);
   sensors.SetRightSonar(0.28);
   sensors.SetLeftSonar(0.1);
   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
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
   obs.Update(sensors, SwarmieAction());
   SwarmieAction a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);

   obs.Update(sensors, SwarmieAction());
   a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);
}

TEST_F(ObstacleBehaviorTest, turnaroundStopsOnlyAfterTimerExpires)
{
   using ::testing::Return;
   EXPECT_CALL(timer, StartOnce()).Times(1);
   EXPECT_CALL(timer, Expired())
      .Times(2)
      .WillOnce(Return(false))
      .WillOnce(Return(true));

   sensors.SetRightSonar(0.1);
   obs.Update(sensors, SwarmieAction());
   SwarmieAction a = obs.GetAction();
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);

   sensors.SetRightSonar(3.2);

   // timer has not expired. keep turning
   obs.Update(sensors, SwarmieAction());
   EXPECT_EQ(0, a.GetVelocity().GetLinearMagnitude());
   EXPECT_TRUE(a.GetVelocity().GetAngularMagnitude() > 0.05);

   // timer expired. no movement.
   obs.Update(sensors, SwarmieAction());
   EXPECT_FALSE(is_moving(obs.GetAction()));
}