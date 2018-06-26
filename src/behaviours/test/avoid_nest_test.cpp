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
   Tag t(NEST_TAG_ID, 1, 1, 1, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_moving(a));
}

/* Tag positions values measured from the robot camera:
 *
 * _____________
 * |A         B|
 * |           |
 * |           |
 * |           |
 * |D         C|
 * -------------
 *
 * A -> x: -0.202 | y: -0.121 | z: 0.5
 * B -> x: 0.213  | y: -0.116 | z: 0.63
 * C -> x: 0.068  | y: 0.042  | z: 0.212
 * D -> xL -0.069 | y: 0.039  | z: 0.63
 */

TEST_F(AvoidNestTest, nestAtTopLeftOfFrameTriggersRightTurn) {
   Tag t(NEST_TAG_ID, -0.202, -0.121, 0.5, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_right(a));
}

TEST_F(AvoidNestTest, nestAtBottomLeftOfFrameTriggersRightTurn) {
   Tag t(NEST_TAG_ID, -0.069, 0.039, 0.63, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_right(a));
}

TEST_F(AvoidNestTest, nestAtTopRightOfFrameTriggersLeftTurn) {
   Tag t(NEST_TAG_ID, 0.213, -0.116, 0.63, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_left(a));
}

TEST_F(AvoidNestTest, nestAtBottomRightOfFrameTriggersLeftTurn) {
   Tag t(NEST_TAG_ID, 0.068, 0.042, 0.212, defaultOrientation);
   sensors.DetectedTag(t);
   avoid.Update();
   Action a = avoid.GetAction();
   EXPECT_TRUE(is_turning_left(a));
}
