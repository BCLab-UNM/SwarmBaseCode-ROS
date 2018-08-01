#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "AvoidNest.hpp"
#include "SwarmieSensors.hpp"
#include "Tag.hpp"
#include "MockTimer.hpp"

class AvoidNestTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   MockTimer timer;
   AvoidNest avoid;
   boost::math::quaternion<double> defaultOrientation;

   AvoidNestTest() : avoid(&sensors, &timer), defaultOrientation(1.2, 1.2, 1.2, 2.1) {
      avoid.Update();
   }
};

TEST_F(AvoidNestTest, noTagsNoAction) {
   SwarmieAction a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AvoidNestTest, nonCenterTagNoAction) {
   Tag t(0, 1, 1, 1, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   SwarmieAction a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AvoidNestTest, nestTagTriggersMovement) {
   EXPECT_CALL(timer, StartOnce()).Times(1);
   Tag t(Tag::NEST_TAG_ID, 1, 1, 1, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   SwarmieAction a = avoid.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(AvoidNestTest, nestAtTopLeftOfFrameTriggersRightTurn) {
   EXPECT_CALL(timer, StartOnce()).Times(1);
   Tag t = tag_top_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   SwarmieAction a = avoid.GetAction();
   EXPECT_TRUE(is_turning_right(a));
}

TEST_F(AvoidNestTest, nestAtBottomLeftOfFrameTriggersRightTurn) {
   EXPECT_CALL(timer, StartOnce()).Times(1);
   Tag t = tag_bottom_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   SwarmieAction a = avoid.GetAction();
   EXPECT_TRUE(is_turning_right(a));
}

TEST_F(AvoidNestTest, nestAtTopRightOfFrameTriggersLeftTurn) {
   EXPECT_CALL(timer, StartOnce()).Times(1);
   Tag t = tag_top_right(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   SwarmieAction a = avoid.GetAction();
   EXPECT_TRUE(is_turning_left(a));
}

TEST_F(AvoidNestTest, nestAtBottomRightOfFrameTriggersLeftTurn) {
   EXPECT_CALL(timer, StartOnce()).Times(1);
   EXPECT_CALL(timer, Expired()).Times(0);
   Tag t = tag_bottom_right(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   SwarmieAction a = avoid.GetAction();
   EXPECT_TRUE(is_turning_left(a));
}

TEST_F(AvoidNestTest, insideNestIgnoreTags) {
   EXPECT_CALL(timer, StartOnce()).Times(0);
   sensors.DetectedTag(negative_yaw_tag(Tag::NEST_TAG_ID));
   avoid.Update();
   EXPECT_FALSE(is_moving(avoid.GetAction()));
   sensors.DetectedTag(tag_top_left(Tag::NEST_TAG_ID));
   avoid.Update();
   EXPECT_FALSE(is_moving(avoid.GetAction()));
}

TEST_F(AvoidNestTest, stillVisibleAfterTurnKeepTurning) {
   using ::testing::Return;
   EXPECT_CALL(timer, StartOnce()).Times(2);
   EXPECT_CALL(timer, Expired()).Times(1)
      .WillOnce(Return(true));

   sensors.DetectedTag(tag_bottom_right(Tag::NEST_TAG_ID));
   avoid.Update();
   avoid.Update();
   EXPECT_TRUE(is_turning_left(avoid.GetAction()));
}

TEST_F(AvoidNestTest, tagsVanishMovementStops)
{
   using ::testing::Return;
   EXPECT_CALL(timer, StartOnce()).Times(1);
   EXPECT_CALL(timer, Expired()).Times(1)
      .WillOnce(Return(true));
   sensors.DetectedTag(tag_bottom_right(Tag::NEST_TAG_ID));
   avoid.Update();
   EXPECT_TRUE(is_turning_left(avoid.GetAction()));
   sensors.ClearDetections();
   avoid.Update();
   EXPECT_FALSE(is_moving(avoid.GetAction()));
}