#include <gtest/gtest.h>
#include <boost/math/quaternion.hpp>

#include "test_util.h"

#include "AvoidNest.hpp"
#include "SwarmieSensors.hpp"
#include "ROSTimer.hpp"

class AvoidNestTest : public testing::Test
{
protected:
   SwarmieSensors sensors;
   AvoidNest<ROSTimer> avoid;
   boost::math::quaternion<double> defaultOrientation;

   AvoidNestTest() : avoid(&sensors), defaultOrientation(1.2, 1.2, 1.2, 2.1) {
      avoid.Update();
   }
};

TEST_F(AvoidNestTest, noTagsNoAction) {
   Action a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AvoidNestTest, nonCenterTagNoAction) {
   Tag t(0, 1, 1, 1, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_FALSE(is_moving(a));
}

TEST_F(AvoidNestTest, nestTagTriggersMovement) {
   Tag t(Tag::NEST_TAG_ID, 1, 1, 1, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_moving(a));
}

TEST_F(AvoidNestTest, nestAtTopLeftOfFrameTriggersRightTurn) {
   Tag t = tag_top_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_right(a));
}

TEST_F(AvoidNestTest, nestAtBottomLeftOfFrameTriggersRightTurn) {
   Tag t = tag_bottom_left(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_right(a));
}

TEST_F(AvoidNestTest, nestAtTopRightOfFrameTriggersLeftTurn) {
   Tag t = tag_top_right(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_left(a));
}

TEST_F(AvoidNestTest, nestAtBottomRightOfFrameTriggersLeftTurn) {
   Tag t = tag_bottom_right(Tag::NEST_TAG_ID);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_left(a));
}

TEST_F(AvoidNestTest, insideNestIgnoreTags) {
   sensors.DetectedTag(negative_yaw_tag(Tag::NEST_TAG_ID));
   avoid.Update();
   EXPECT_FALSE(is_moving(avoid.GetAction()));
   sensors.DetectedTag(tag_top_left(Tag::NEST_TAG_ID));
   avoid.Update();
   EXPECT_FALSE(is_moving(avoid.GetAction()));
}
