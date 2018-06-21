#include "BehaviorManager.hpp"
#include "SwarmieInterface.hpp"
#include "ObstacleBehavior.hpp"

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

bool is_moving(Action a)
{
   return (fabs(a.drive.left) >= 35 || fabs(a.drive.right) >= 35);
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

TEST(ObstacleBehavior, allFar)
{
   SwarmieSensors sensors;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);

   ObstacleBehavior obs(&sensors);
   obs.Update();
   Action a = obs.GetAction();
   EXPECT_LT(a.drive.left, 0.1);
   EXPECT_LT(a.drive.right, 0.1);
}

// TODO: Test the of ObstacleBehavior behavior when each US is at a
// mid/low value (0.8, 0.7, 0.5, 0.3) In each case the movement should
// be non-negligible
TEST(ObstacleBehavior, leftSonarTriggersMovement)
{
   SwarmieSensors sensors;
   Action a;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);

   ObstacleBehavior obs(&sensors);
   obs.Update();

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

TEST(ObstacleBehavior, rightSonarTriggersMovement)
{
   SwarmieSensors sensors;
   Action a;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);

   ObstacleBehavior obs(&sensors);
   obs.Update();

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

TEST(ObstacleBehavior, centerSonarTriggersMovement)
{
   SwarmieSensors sensors;
   Action a;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);

   ObstacleBehavior obs(&sensors);
   obs.Update();

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
TEST(ObstacleBehavior, leftTriggersTurnaround)
{
   Action a;
   SwarmieSensors sensors;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);
   ObstacleBehavior obs(&sensors);
   obs.Update();

   sensors.SetLeftSonar(0.29);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST(ObstacleBehavior, rightTriggersTurnaround)
{
   Action a;
   SwarmieSensors sensors;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);
   ObstacleBehavior obs(&sensors);
   obs.Update();

   sensors.SetRightSonar(0.29);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));
}

TEST(ObstacleBehavior, centerSonarTriggersTurnaround)
{
   Action a;
   SwarmieSensors sensors;
   sensors.SetLeftSonar(3.2);
   sensors.SetRightSonar(3.2);
   sensors.SetCenterSonar(3.2);
   ObstacleBehavior obs(&sensors);
   obs.Update();

   sensors.SetCenterSonar(0.29);
   obs.Update();
   a = obs.GetAction();
   ASSERT_NE(a.drive.left, 0);
   ASSERT_NE(a.drive.right, 0);
   EXPECT_EQ(a.drive.left, -(a.drive.right));   
}

int main(int argc, char** argv)
{
   testing::InitGoogleTest(&argc, argv);
   ros::init(argc, argv, "tester");
   return RUN_ALL_TESTS();
}
